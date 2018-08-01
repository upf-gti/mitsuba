/* ---------------------------------------------------------------------------
** videoHDR.cpp
** This file implements the VideoHDR integrator which generates an image where
** each pixel's value corresponds to the irradiance at a randomly selected 3D point
**
** Author: Ricardo Marques
** Date: March, 2014
** -------------------------------------------------------------------------*/

#include <mitsuba/render/scene.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/render/videoHDR_RecordMap.h>
//#include <mitsuba/core/fwd.h>

MTS_NAMESPACE_BEGIN


/* Class prototype */
class VideoHDRIntegrator : public SamplingIntegrator {
public:
	// Constructor
	VideoHDRIntegrator(const Properties &props);
	/// Unserialize from a binary data stream
	VideoHDRIntegrator(Stream *stream, InstanceManager *manager);
	void serialize(Stream *stream, InstanceManager *manager) const;
	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID);
	void configure();
	void configureSampler(const Scene *scene, Sampler *sampler);
	void addChild(const std::string &name, ConfigurableObject *child) ;
	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const;
	std::string toString() const;
	/* My Methods */
	Spectrum colorCode(int min, int max, int value) const;
	bool render(Scene *scene,
		RenderQueue *queue, const RenderJob *job,
		int sceneResID, int sensorResID, int samplerResID);

	MTS_DECLARE_CLASS()
private:
	/* Private Methods */
	Vector randomRotationArroundY( Vector dir, float sinAlpha, float cosAlpha) const;
	//Vector VideoHDRIntegrator::sphericalToCartesian(float theta, float phi) const;
	//void VideoHDRIntegrator::prepareSpiralSS(unsigned int nSamplesBSDF);
	Vector sphericalToCartesian(float theta, float phi) const;
	void prepareSpiralSS(unsigned int nSamplesBSDF);
	/* Attributes */
	ref<SamplingIntegrator> m_subIntegrator;
	DiscreteDistribution m_shapesPDF;
	size_t m_samples;
	//std::vector<Vector> m_spiralSampleSet;
	std::vector<VideoHDR_Record*> m_hdrRecordList;
	bool m_computeDirectComponent;
	bool m_saveRecordsObj;
	bool m_showReconstruction;
	bool m_firstRenderingPass;
	bool m_sampleAccordingBRDF;
	ref<videoHDR_RecordMap> m_videoHDR_RecordMap;
};

/* ************************* */
/* Public Methods Definition */
/* ************************* */

VideoHDRIntegrator::VideoHDRIntegrator(const Properties &props) : SamplingIntegrator(props) {
	m_samples = props.getSize("samples", 1);
	m_computeDirectComponent = props.getBoolean("computeDirectComponent", false);
	m_saveRecordsObj = props.getBoolean("saveRecordsObj", false);
	m_showReconstruction = props.getBoolean("showReconstruction", false);
	m_sampleAccordingBRDF = props.getBoolean("sampleAccordingBRDF", false);
	m_firstRenderingPass = true;
}

	/// Unserialize from a binary data stream
VideoHDRIntegrator::VideoHDRIntegrator(Stream *stream, InstanceManager *manager) : SamplingIntegrator(stream, manager) {
	configure();
}

void VideoHDRIntegrator::serialize(Stream *stream, InstanceManager *manager) const {
	SamplingIntegrator::serialize(stream, manager);
}

bool VideoHDRIntegrator::preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID) {
	SamplingIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
	
	m_videoHDR_RecordMap = new videoHDR_RecordMap(m_samples);
	// Some local variables (for randomly chosing 3D points on the scene)
	float sample1D;
	Point2 sample2D;
	size_t shapeIndex;
	ref_vector<Shape> shapes = scene->getShapes();
	
	// Call the preprocess method of the nested integrator
	m_subIntegrator->preprocess(scene, queue, job, 
		sceneResID, sensorResID, samplerResID);
	// Create a pdf for sampling the shapes
	m_shapesPDF.clear();
	if (shapes.size() == 0) {
		Log(EWarn, "No shapes found.");
	}
	// Calculate a discrete PDF to importance sample emitters
	for (ref_vector<Shape>::iterator it = shapes.begin(); it != shapes.end(); ++it) {
		m_shapesPDF.append(it->get()->getSurfaceArea() );
	}
	m_shapesPDF.normalize();

	/* **************************************** */
	/* Randomly select a 3D point on the scene  */
	/* and store them in a videoHDR_Record list */
	/* **************************************** */
	// Step 1: Create an Independent Sampler
	ref<Sampler> sampler = static_cast<Sampler *> (PluginManager::getInstance()->
		createObject(MTS_CLASS(Sampler), Properties("independent")));
	const Vector2i filmSize = scene->getFilm()->getSize();
	int nPixels = filmSize.x * filmSize.y;
	for(int i=0; i<nPixels; i++) {
		// Step 2: Randomly select a shape
		sample1D = sampler->next1D();
		shapeIndex = m_shapesPDF.sample(sample1D);
		size_t numberShapes = m_shapesPDF.size();
		// Step 3: Randomly select a 3D point on the selected shape
		sample2D = sampler->next2D();
		PositionSamplingRecord p;
		shapes.at(shapeIndex)->samplePosition(p, sample2D);

		// Create the hdr record
		Spectrum fake_spec = Spectrum(-1.f);
		VideoHDR_Record *hdrRecord = new VideoHDR_Record(p.p, fake_spec, shapeIndex, p.n);
		m_hdrRecordList.push_back(hdrRecord);
	}

	// To Do? (Compute the spiral points samples set)
	//prepareSpiralSS(m_samples);

	return true;
}

void VideoHDRIntegrator::configure() {
	SamplingIntegrator::configure();
	//m_subIntegrator->configure();
}

void VideoHDRIntegrator::configureSampler(const Scene *scene, Sampler *sampler) {

	SamplingIntegrator::configureSampler(scene, sampler);

	//sampler->request1DArray(1);
	//sampler->request2DArray(1);
	//m_subIntegrator->configureSampler(scene, sampler);

	for(size_t i=0; i<m_samples; i++) {
		m_subIntegrator->configureSampler(scene, sampler);
	}

	sampler->request2DArray(m_samples);
}

void VideoHDRIntegrator::addChild(const std::string &name, ConfigurableObject *child) {
	const Class *cClass = child->getClass();

	if (cClass->derivesFrom(MTS_CLASS(Integrator))) {
		if (!cClass->derivesFrom(MTS_CLASS(SamplingIntegrator)))
			Log(EError, "The sub-integrator must be derived from the class SamplingIntegrator");
		m_subIntegrator = static_cast<SamplingIntegrator *>(child);
		m_subIntegrator->setParent(this);
	} else {
		Integrator::addChild(name, child);
	}
}

Spectrum VideoHDRIntegrator::colorCode(int min, int max, int value) const {

	float red, green, blue;
	// Compute the mean value
	float mean = (max - min) / 2.f;
	// Compute the slope for Blue and Red
	float betaRB = 1.f / (max - mean);
	float betaG = 2.f * betaRB;

	if(min == max) {
		red = 0.f;
		green = 1.f;
		blue = 0.f;
	} else {

		if(value < mean) {
			red = 0.f;
			blue = 1.f - betaRB * value;
			if(value <= mean/2.f) {
				green = 0.f;
			} else {
				green = value - (mean/2.f) * betaG;
			}
		} else {
			red = (value - mean) * betaRB;
			blue = 0.f;
			if(value <= 3.f / 4.f * mean) {
				green = 1 - (value - mean) * betaG;
			} else {
				green = 0.f;
			}
		}
	}
	Spectrum color;
	color.fromLinearRGB(red, green, blue);

	return color;
}

bool VideoHDRIntegrator::render(Scene *scene,
		RenderQueue *queue, const RenderJob *job,
		int sceneResID, int sensorResID, int samplerResID) {

	Log(EInfo, "First Rendering Pass.");
	m_firstRenderingPass = true;
	SamplingIntegrator::render(scene, queue, job, sceneResID, sensorResID, samplerResID);

	// Build the KdTree
	for(int i=0; i<m_hdrRecordList.size(); i++) {
		m_videoHDR_RecordMap->push_back(*m_hdrRecordList.at(i));
	}
	m_videoHDR_RecordMap->build();

	// Save the records in an obj
	if(m_saveRecordsObj) {
		std::stringstream ss;
		ss << scene->getDestinationFile().string() << "_HDR_records.obj";
		//const std::string myString = scene->getDestinationFile().c_str();
		std::string fileObj = ss.str();
		Log(EInfo, "Write out the OBJ record files: %s", fileObj.c_str());
		m_videoHDR_RecordMap->dumpOBJ(fileObj);
	}
	
	if(m_showReconstruction) {
		Film *film = scene->getFilm();
		film->develop(scene, 0.f);
		film->clear();
		
		Log(EInfo, "Second Rendering Pass.");
		m_firstRenderingPass = false;

		std::stringstream ss;
		ss << scene->getDestinationFile().string() << "_Reconstructed";
		std::string fileReconstruct = ss.str();
		
		film->setDestinationFile(fileReconstruct.c_str(), 0);
		SamplingIntegrator::render(scene, queue, job, sceneResID, sensorResID, samplerResID);
	}

	return true;
	
}

Spectrum VideoHDRIntegrator::Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {
	
	Intersection its;
	const BSDF *bsdf;
	const Scene *scene = rRec.scene;

	if(m_firstRenderingPass) {
		Spectrum Li(0.f);
		Point2 sample2D;
		

		/* ***************************************************** */
		/* Fetch the pixel which gave origin to this primary ray */
		/* and fetch the corresponding hdr video record from the */
		/* list of recrods										 */
		/* ***************************************************** */
		Intersection itsTemp;
		itsTemp.p = r.o + 1000.f*r.d;
		PositionSamplingRecord pRec(itsTemp);
		DirectionSamplingRecord dRec(r.d);
		Point2 pixelPos;
		if(!rRec.scene->getSensor()->getSamplePosition(pRec, dRec, pixelPos)) {
			Log(EWarn, "Impossible to compute the pixel through which this ray was shot!");
			Log(EWarn, "Returning Spectrum(-1.f)");
			return Spectrum(-1.f);
		}
		Vector2i size = rRec.scene->getFilm()->getSize();
		int pixelIndex = (int)floor(pixelPos.y) * size.x + (int)floor(pixelPos.x);
		VideoHDR_Record *hdrRecord = m_hdrRecordList.at(pixelIndex);

		// Request sample array for BRDF sampling
		Point2 *sampleArray;
		if (m_samples > 1) {
			sampleArray = rRec.sampler->next2DArray(m_samples);
		} else {
			sample2D = rRec.nextSample2D(); 
			sampleArray = &sample2D;
		}

		/* ****************************** */
		/* Sample Incident Radiance using */
		/* a random variable wo ~ BSDF    */
		/* ****************************** */
		unsigned int loopCount = 0;
		bool badPoint = false;
		do {
			loopCount ++;

			/* ************************************************************** */
			/* This part is all about setting up the structure "Intersection" */
			/* ************************************************************** */
			Point p = hdrRecord->getPosition();
			Normal n = hdrRecord->getNormal();
			its.p = p;
			its.geoFrame = Frame(n);
			its.shFrame = Frame(n);
			its.wi = its.toLocal(n);

			bool isEmitter = scene->getShapes().at(hdrRecord->data.shapeIndex)->isEmitter();
			
			if(!isEmitter) {

				// Set up the BRDF
				bsdf = scene->getShapes().at(hdrRecord->data.shapeIndex)->getBSDF();
				// Set the wi to the normal at intersection point
				its.wi = its.toLocal(its.shFrame.n);
				rRec.its = its;

				for(unsigned int i=0; i<m_samples; i++) {
					Float bsdfPdf;
					BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
					Spectrum bsdfVal;

					if(m_sampleAccordingBRDF) {
						/* Sample BSDF * cos(theta) and also request the local probability density */
						bsdfVal = bsdf->sample(bRec, bsdfPdf, sampleArray[i]);
						if(bsdfPdf == 0)
							continue;
					} else {
						/* Sample from a cosine lobe centered around (0, 0, 1) */
						float phi  = sampleArray[i].x * 2.f * M_PI;
						float theta  = std::acos(sampleArray[i].y);
						bRec.wo = sphericalToCartesian(theta, phi);
						bsdfVal = Spectrum(1.f);
						//bsdfVal = bsdf->getDiffuseReflectance();
					}

					RadianceQueryRecord rRec2;
					rRec2.recursiveQuery(rRec);
					if (!m_computeDirectComponent) 
						rRec2.type = RadianceQueryRecord::ERadianceNoEmission;
					// Obtain the incident radiance from direction XX
					RayDifferential secondaryRay(its.p, its.toWorld(bRec.wo), r.time);
					Spectrum Li_secondaryRay = m_subIntegrator->Li(secondaryRay, rRec2);
					Li += Li_secondaryRay * bsdfVal;
				}

				Li = Li/(float)m_samples;
				//badPoint = Li.isZero();
				badPoint = (Li.average() < 0.0001f);
			} else {
				badPoint = true;
			}
			
			if(badPoint) {
				if(loopCount % 2 == 0 || isEmitter) {
					// Step 1: Randomly select a shape
					float sample1D = rRec.sampler->next1D();
					size_t shapeIndex = m_shapesPDF.sample(sample1D);
					size_t numberShapes = m_shapesPDF.size();
					// Step 2: Randomly select a 3D point on the selected shape
					Point2 sample2D = rRec.sampler->next2D();
					PositionSamplingRecord p;
					scene->getShapes().at(shapeIndex)->samplePosition(p, sample2D);
					// Change the hdr record
					hdrRecord->setPosition(p.p);
					hdrRecord->data.n = p.n;
					hdrRecord->data.shapeIndex = shapeIndex;
				} else {
					hdrRecord->data.n = -hdrRecord->data.n;
				}
				Li = Spectrum(0.f);
			}
		} while(badPoint);

		/*if(loopCount > 1)
			Log(EInfo, "I was a bad pixel which becme good after %d iterations!", loopCount);*/

		if(badPoint) {
			float badColor[3] = {0.f, 0.f, 1.f};
			Li = Spectrum(badColor);
		}

		hdrRecord->setIrradiance(Li);
		return Li;

	} else {
		if (!rRec.rayIntersect(r)) {
			/* If no intersection could be found, possibly return
			radiance from a background emitter */
			if (rRec.type & RadianceQueryRecord::EEmittedRadiance & m_computeDirectComponent)
				return scene->evalEnvironment(r);
			else
				return Spectrum(0.0f);
		}
		its = rRec.its;
		// Querry the Kd-tree
		size_t k = 1;
		videoHDR_RecordMap::SearchResult *results = new videoHDR_RecordMap::SearchResult[k+1];
		m_videoHDR_RecordMap->nnSearch(its.p, k, results);
		const VideoHDR_Record& rec = (*m_videoHDR_RecordMap)[results[0].index];
		Spectrum Li(0.f);
		return rec.getIrradiance();
	}
}


std::string VideoHDRIntegrator::toString() const {
	std::ostringstream oss;
	oss << "VideoHDRIntegrator[" << endl
		<< "]";
	return oss.str();
}

/* ************************** */
/* Private Methods Definition */
/* ************************** */

/// Rotate a sample arround Y by a random angle
Vector VideoHDRIntegrator::randomRotationArroundY( Vector dir, float sinAlpha, float cosAlpha) const {

	Vector rotatedDir = Vector( dir.x * cosAlpha + dir.z * sinAlpha, 
								dir.y, 
							   -dir.x * sinAlpha + dir.z * cosAlpha );
	return rotatedDir;
}

/// Converts Spherical Coordinates to Cartesian, According to the MISTUBA Referential (Y up)
Vector VideoHDRIntegrator::sphericalToCartesian(float theta, float phi) const {
	// Express the Spherical direction in the local frame
	Vector cartesian;
	cartesian.x = sinf(theta) * cosf(phi);
	cartesian.y = sinf(theta) * sinf(phi);
	cartesian.z = cosf(theta);
	return cartesian;
}

/// Distributes "nBaseSamples" points on a BSDF of shininess "shininess"
/*void VideoHDRIntegrator::prepareSpiralSS(unsigned int nSamplesBSDF) {

	std::stringstream outInfo;
	//outInfo << "Preparing the BRDF " << getName().c_str() << " for using Spiral Points Sampling";
	Log(EInfo, outInfo.str().c_str());

	// Spiral Points Algorithm with first point in the center of the lobe
	float thetaDir, phiDir;

	// Vector of Directions
	//std::vector<Vector> baseSampleSet;
	m_spiralSampleSet.reserve(nSamplesBSDF);

	// Initialize Variables
	float d_phiAux = M_PI * (3.f - sqrt(5.f));
	float phiAux = 0.f;
	float d_zAux = 1.f / nSamplesBSDF;
	//float zAux = 1.f;
	float zAux = 1.f - d_zAux/2.f;

	// Spiral Points for lobe
	for(unsigned int k = 0; k < nSamplesBSDF; k++) 
	{
		//thetaDir = acosf( zAux );
		//thetaDir = acosf( powf(zAux,(1.f/(phongExponent+1.f))) );
		thetaDir = acosf( powf(zAux,(1.f/2.f)) );
		phiDir   = fmod(phiAux, 2.f * M_PI);
		zAux     = zAux - d_zAux;
		phiAux   = phiAux + d_phiAux;

		// Transform Spherical Coordinates in Euclidean Coordinates
		Vector d = sphericalToCartesian(thetaDir, phiDir);
		m_spiralSampleSet.push_back( normalize(d) );
	}
}*/

MTS_IMPLEMENT_CLASS_S(VideoHDRIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(VideoHDRIntegrator, "Video HDR Integrator");
MTS_NAMESPACE_END
