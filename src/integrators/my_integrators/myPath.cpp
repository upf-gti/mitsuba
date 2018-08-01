/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);


class MYPathTracer : public MonteCarloIntegrator {
public:

	MYPathTracer(const Properties &props)
		: MonteCarloIntegrator(props) {

		/*Assign the the values of the variables named "showOnePath", "pathRangeMin" and "pathRangeMax" 
		* in the .xml file to the variables m_onePath, m_minPath and m_maxPath respectively. 
		* If no value was set, the default will be -1 */

		m_onePath = props.getInteger("showOnePath", -1);
		m_minPath = props.getInteger("pathRangeMin", -1);
		m_maxPath = props.getInteger("pathRangeMax", -1);
	}

	/// Unserialize from a binary data stream
	MYPathTracer(Stream *stream, InstanceManager *manager)
		: MonteCarloIntegrator(stream, manager) {

		m_onePath = stream->readInt();
		m_minPath = stream->readInt();
		m_maxPath = stream->readInt();
	}


	/* Function that can be used to know whether a ray needs to traced or not (show will be 0)
		or to know whether a ray needs to be shown in the final render or not (show will be 1) */
	inline bool computeRay(int depth, bool show=0) const {

		bool result;

		switch (path_to_show) {
			case EAll:
				result = (depth <= m_maxDepth || m_maxDepth < 0) ? true : false;
				break;
				
			case EOne:
				if (show)
					result = (depth == m_onePath) ? true : false;
				else
					result = (depth <= m_onePath) ? true : false;
				break;

			case ELimitedRange:
				if (show)
					result = (depth >= m_minPath && depth <=m_maxPath) ? true : false;
				else
					result = (depth <= m_maxPath) ? true : false;
				break;

			case ELowRange:
				result = (depth <= m_maxPath) ? true : false;
				break;

			case EHighRange:
				if (show)
					result = (depth >= m_minPath) ? true : false;
				else
					result = (depth <= m_maxDepth || m_maxDepth < 0) ? true : false;
				break;

			default:
				Log(EError, "Unknown case");
				break;

		}

		return result;
	}


	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {

		/* boolean that will determine if a path depth will be shown in the final render or not */
		bool contributes;

		/* Some aliases and local variables */
		const Scene *scene = rRec.scene;
		Intersection &its = rRec.its;
		RayDifferential ray(r);
		Spectrum Li(0.0f);
		bool scattered = false;

		/* Perform the first ray intersection (or ignore if the
		   intersection has already been provided). */
		rRec.rayIntersect(ray);
		ray.mint = Epsilon;

		Spectrum throughput(1.0f);
		Float eta = 1.0f;

		while (computeRay(rRec.depth)){

			contributes = computeRay(rRec.depth, 1);

			if (!its.isValid()) {
				/* If no intersection could be found, potentially return
				   radiance from a environment luminaire if it exists */
				if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
					&& (!m_hideEmitters || scattered) && contributes) 

						Li += throughput * scene->evalEnvironment(ray);	
				
				break;
			}


			const BSDF *bsdf = its.getBSDF(ray);

			/* Possibly include emitted radiance if requested */
			if (its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance)
				&& (!m_hideEmitters || scattered) && contributes)
								
					Li += throughput * its.Le(-ray.d);


			/* Include radiance from a subsurface scattering model if requested */
			if (its.hasSubsurface() && (rRec.type & RadianceQueryRecord::ESubsurfaceRadiance) && contributes)
					
					Li += throughput * its.LoSub(scene, rRec.sampler, -ray.d, rRec.depth);
				

			if ((rRec.depth >= m_maxDepth && m_maxDepth > 0)
				|| (m_strictNormals && dot(ray.d, its.geoFrame.n)
					* Frame::cosTheta(its.wi) >= 0)) {

				/* Only continue if:
				   1. The current path length is below the specifed maximum
				   2. If 'strictNormals'=true, when the geometric and shading
				      normals classify the incident direction to the same side */
				break;
			}

			/* ==================================================================== */
			/*                     Direct illumination sampling                     */
			/* ==================================================================== */

			/* Estimate the direct illumination if this is requested */
			DirectSamplingRecord dRec(its);

			if (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance &&
				(bsdf->getType() & BSDF::ESmooth)) {
				Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());
				if (!value.isZero()) {
					const Emitter *emitter = static_cast<const Emitter *>(dRec.object);

					/* Allocate a record for querying the BSDF */
					BSDFSamplingRecord bRec(its, its.toLocal(dRec.d), ERadiance);

					/* Evaluate BSDF * cos(theta) */
					const Spectrum bsdfVal = bsdf->eval(bRec);

					/* Prevent light leaks due to the use of shading normals */
					if (!bsdfVal.isZero() && (!m_strictNormals
							|| dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

						/* Calculate prob. of having generated that direction
						   using BSDF sampling */
						Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
							? bsdf->pdf(bRec) : 0;

						/* Weight using the power heuristic */
						Float weight = miWeight(dRec.pdf, bsdfPdf);
						
						if (contributes)
							Li += throughput * value * bsdfVal * weight;
					}
				}
			}

			/* ==================================================================== */
			/*                            BSDF sampling                             */
			/* ==================================================================== */

			/* Sample BSDF * cos(theta) */
			Float bsdfPdf;
			BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
			Spectrum bsdfWeight = bsdf->sample(bRec, bsdfPdf, rRec.nextSample2D());
			if (bsdfWeight.isZero())
				break;

			scattered |= bRec.sampledType != BSDF::ENull;

			/* Prevent light leaks due to the use of shading normals */
			const Vector wo = its.toWorld(bRec.wo);
			Float woDotGeoN = dot(its.geoFrame.n, wo);
			if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
				break;

			bool hitEmitter = false;
			Spectrum value;

			/* Trace a ray in this direction */
			ray = Ray(its.p, wo, ray.time);
			if (scene->rayIntersect(ray, its)) {
				/* Intersected something - check if it was a luminaire */
				if (its.isEmitter()) {
					value = its.Le(-ray.d);
					dRec.setQuery(ray, its);
					hitEmitter = true;
				}
			} else {
				/* Intersected nothing -- perhaps there is an environment map? */
				const Emitter *env = scene->getEnvironmentEmitter();

				if (env) {
					if (m_hideEmitters && !scattered)
						break;

					value = env->evalEnvironment(ray);
					if (!env->fillDirectSamplingRecord(dRec, ray))
						break;
					hitEmitter = true;
				} else {
					break;
				}
			}

			/* Keep track of the throughput and relative
			   refractive index along the path */
			throughput *= bsdfWeight;
			eta *= bRec.eta;

			/* If a luminaire was hit, estimate the local illumination and
			   weight using the power heuristic and show if we only want 
				   the direct light (without any bounces) */
			if (hitEmitter &&
				(rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance)) {
				/* Compute the prob. of generating that direction using the
				   implemented direct illumination sampling technique */
				const Float lumPdf = (!(bRec.sampledType & BSDF::EDelta)) ?
					scene->pdfEmitterDirect(dRec) : 0;

				if (contributes)							
					Li += throughput * value * miWeight(bsdfPdf, lumPdf);
			}

			/* ==================================================================== */
			/*                         Indirect illumination                        */
			/* ==================================================================== */

			/* Set the recursive query type. Stop if no surface was hit by the
			   BSDF sample or if indirect illumination was not requested */
			if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
				break;
			rRec.type = RadianceQueryRecord::ERadianceNoEmission;

			if (rRec.depth++ >= m_rrDepth) {
				/* Russian roulette: try to keep path weights equal to one,
				   while accounting for the solid angle compression at refractive
				   index boundaries. Stop with at least some probability to avoid
				   getting stuck (e.g. due to total internal reflection) */

				Float q = std::min(throughput.max() * eta * eta, (Float) 0.95f);		
				if (rRec.nextSample1D() >= q)
					break;
				throughput /= q;
			}
		}

		/* Store statistics */
		avgPathLength.incrementBase();
		avgPathLength += rRec.depth;


		return Li;
	}


	inline Float miWeight(Float pdfA, Float pdfB) const {
		pdfA *= pdfA;
		pdfB *= pdfB;
		return pdfA / (pdfA + pdfB);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		MonteCarloIntegrator::serialize(stream, manager);
		stream->writeInt(m_onePath);
		stream->writeInt(m_minPath);
		stream->writeInt(m_maxPath);
	}

	// Preprocess function -- called on the initiating machine
    bool preprocess(const Scene *scene, RenderQueue *queue, 
        const RenderJob *job, int sceneResID, int cameraResID, 
        int samplerResID) {
        
        SamplingIntegrator::preprocess(scene, queue, job, sceneResID,
        cameraResID, samplerResID);

        //Preprocess scene file to avoid compilation errors and to define the variable path_to_show
        if (m_onePath != -1) {
        	if (m_minPath == -1 && m_maxPath == -1) {
        		path_to_show = EOne;
        		return true;
        	}

        	Log(EError, "Choose between showing one path size or a range. Cannot do both");
        }
        
    	if (m_maxPath != -1) {
    		if (m_maxDepth != -1 && m_maxPath > m_maxDepth)
    			Log(EError, "pathRangeMax can't be higher than maxDepth");

    		if (m_minPath != -1) {
    			if (m_minPath < m_maxPath){

    				path_to_show = ELimitedRange;
    				return true;
    			}

    			Log(EError, "pathRangeMin can't be higher than pathRangeMax");
    		}

    		path_to_show = ELowRange;
    		return true;
    	}

    	if (m_minPath != -1) {

    		path_to_show = EHighRange;
    		return true;
    	}

    	path_to_show = EAll;
    	return true;

    }

	std::string toString() const {
		std::ostringstream oss;
		oss << "MYPathTracer[" << endl
			<< "  maxDepth = " << m_maxDepth << "," << endl
			<< "  rrDepth = " << m_rrDepth << "," << endl
			<< "  showOnePath = " << m_onePath << "," << endl
			<< "  pathRangeMin = " << m_minPath << "," << endl
			<< "  pathRangeMax = " << m_maxPath << "," << endl
			<< "  strictNormals = " << m_strictNormals << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS()

private:

	enum EPathToShow{
		EAll = 0,
		EOne = 1,
		ELimitedRange = 2,
		ELowRange = 3,
		EHighRange = 4
	};

	EPathToShow path_to_show;
	int m_onePath;
	int m_minPath;
	int m_maxPath;
};

MTS_IMPLEMENT_CLASS_S(MYPathTracer, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(MYPathTracer, "MY path tracer");
MTS_NAMESPACE_END
