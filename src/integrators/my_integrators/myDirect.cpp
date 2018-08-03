#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

class MyDirect : public SamplingIntegrator {

public:
    MTS_DECLARE_CLASS()
    
    // Initialize the integrator with the specified properties
    MyDirect(const Properties &props) : SamplingIntegrator(props) {
    
    	m_emitterSamples = props.getInteger("emitterSamples",10);

    }
    
    // Unserialize from a binary data stream
    MyDirect(Stream *stream, InstanceManager *manager)
        : SamplingIntegrator(stream, manager) {
        
        m_emitterSamples = stream->readInt();
        configure();
       
    }
    
    // Serialize to a binary data stream
    void serialize(Stream *stream, InstanceManager *manager) const{
        
        SamplingIntegrator::serialize(stream, manager);
        stream->writeInt(m_emitterSamples);
    }
    
    void configureSampler(const Scene *scene, Sampler *sampler) {
		
		SamplingIntegrator::configureSampler(scene, sampler);
        
        /* Allocate space for an array of size m_emitterSamples of 2D samples*/
		if (m_emitterSamples > 1)
			sampler->request2DArray(m_emitterSamples);
	}
    
    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{
    	
    	Spectrum Li(0.0f);
    	const Scene *scene = rRec.scene;
    	Intersection &its = rRec.its;
    	RayDifferential ray(r);
    	Point2 sample;
    	
    	if (rRec.rayIntersect(ray) && !its.isEmitter()) {
    	/* Only trace the ray if it intersects an object which does not emit light*/
            
    		const ref_vector<Emitter> lightSources = scene->getEmitters();
    		
    		for (int i=0; i<lightSources.size(); ++i) {
    			/* Throw a ray to each of the lights in the scene */
                
    			if (lightSources[i]->getProperties().getPluginName() == "point") {
    				Log(EInfo, "***** POINT LIGHT SOURCE *****");
    			}
    			
    			else if (lightSources[i]->getProperties().getPluginName() == "area") {
    				
    				Point2 *sampleArray;
    				
    				if (m_emitterSamples > 1){
    				//Log(EInfo, "%d samples allocated for lightsource #%d", m_emitterSamples, i);
    					sampleArray = rRec.sampler->next2DArray(m_emitterSamples);
    				}
    				
    				else if (m_emitterSamples == 1){
    					sample = rRec.nextSample2D();
    					sampleArray = &sample;
    				}
    				
    				DirectSamplingRecord dRec(its);
                    
                    for (int i=0; i<m_emitterSamples; ++i) {
                        /* Sample a position on the light source which contributes to the point */
    					Spectrum emitter_value = scene->sampleEmitterDirect(dRec, sampleArray[i]);
    					
    					if (!emitter_value.isZero()) {
                            /* Evaluate the BSDF for a light coming from the sampled light source and observed from the origin of the traced ray  */
    						BSDFSamplingRecord BSDFRec(its, its.toLocal(dRec.d));
    						Spectrum BSDF_value = its.getBSDF()->eval(BSDFRec);
    						
    						if (!BSDF_value.isZero()) {
    							Li+=emitter_value*BSDF_value;
    						}    						
    	 				}    				
    				}
    			}
    		}
    	}
    	
    	return Li/m_emitterSamples;
    }
    
    // Preprocess function -- called on the initiating machine
    bool preprocess(const Scene *scene, RenderQueue *queue, 
        const RenderJob *job, int sceneResID, int cameraResID, 
        int samplerResID) {
        
        SamplingIntegrator::preprocess(scene, queue, job, sceneResID,
        cameraResID, samplerResID);
        
        ref_vector<Emitter> lights = scene->getEmitters();
        
        /* This integrator will only work with point and area lights. Throw error if there are other types of emitter */
        for (int i=0; i<lights.size(); ++i){
        	if (lights[i]->getProperties().getPluginName()!= "point" &&
        		lights[i]->getProperties().getPluginName()!= "area")
        	{
        		Log(EError, "Lights of type '%s' not supported", lights[i]->getProperties().getPluginName().c_str());
        	}
        }                
        return true;
    } 
    
private:
    
	int m_emitterSamples;
};

MTS_IMPLEMENT_CLASS_S(MyDirect, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(MyDirect, "Simpler implementation of a direct integrator");

MTS_NAMESPACE_END
