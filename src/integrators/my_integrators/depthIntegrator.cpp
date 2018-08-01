#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

class DepthIntegrator : public SamplingIntegrator {

public:
    MTS_DECLARE_CLASS()
    
    // Initialize the integrator with the specified properties
    DepthIntegrator(const Properties &props) : SamplingIntegrator(props) { }
    
    // Unserialize from a binary data stream
    DepthIntegrator(Stream *stream, InstanceManager *manager)
        : SamplingIntegrator(stream, manager) {
        m_maxDist = stream->readFloat();
    }
    
    // Serialize to a binary data stream
    void serialize(Stream *stream, InstanceManager *manager) const{
        SamplingIntegrator::serialize(stream, manager);
        stream->writeFloat(m_maxDist);
    }
    
    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{
        if (rRec.rayIntersect(r)) {

            Float distance = rRec.its.t;
            //Invert the number so that the far points are darker than the close ones
            return Spectrum(1.0f - distance/m_maxDist);
        }
        return Spectrum(0.0f); 
    }
    
    // Preprocess function -- called on the initiating machine
    bool preprocess(const Scene *scene, RenderQueue *queue, 
        const RenderJob *job, int sceneResID, int cameraResID, 
        int samplerResID) {
        
        SamplingIntegrator::preprocess(scene, queue, job, sceneResID,
        cameraResID, samplerResID);
        
        //Get the bounding box of the whole scene
        const AABB &sceneAABB = scene->getAABB();
        
        // Find the camera position at t=0 seconds
        Point cameraPosition = scene->getSensor()->getWorldTransform()->eval(0).transformAffine(Point(0.0f));
        m_maxDist = - std::numeric_limits<Float>::infinity();
        
        //Check the distance of all corners from the scene bounding box with the camera position. 
        //The maximum distance will be stored
        for (int i=0; i<8; ++i){
            m_maxDist = std::max(m_maxDist, (cameraPosition - sceneAABB.getCorner(i)).length());
        }   

        std::cout<<"Max Dist: "<<m_maxDist<<std::endl;
        return true;
    } 
        
        
private:
    Float m_maxDist;
	
};

MTS_IMPLEMENT_CLASS_S(DepthIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(DepthIntegrator, "Depth integrator");

MTS_NAMESPACE_END
