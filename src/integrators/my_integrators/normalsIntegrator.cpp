#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

class NormIntegrator : public SamplingIntegrator {

public:
    MTS_DECLARE_CLASS()
    
    // Initialize the integrator with the specified properties
    NormIntegrator(const Properties &props) : SamplingIntegrator(props) {
    	
    }
    
    // Unserialize from a binary data stream
    NormIntegrator(Stream *stream, InstanceManager *manager)
        : SamplingIntegrator(stream, manager) {
       
    }
    
    // Serialize to a binary data stream
    void serialize(Stream *stream, InstanceManager *manager) const{
        SamplingIntegrator::serialize(stream, manager);
    }
    
    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const{
    	
        //Initialize the color of the pixel to black
    	Spectrum normal_color(0.0f);
        
        if (rRec.rayIntersect(r)){
            
            //get the normal vector at the intersection point
        	Normal normal = rRec.its.shFrame.n;
        
			if (normal.x < -1)	normal.x = -1;
			else if (normal.y < -1)	normal.y = -1;
			else if (normal.z < -1)	normal.z = -1;
		
            //Transform values from [-1, 1] to [0, 1]
			normal_color.fromLinearRGB((normal.x+1)/2, (normal.y+1)/2, (normal.z+1)/2);
        }
        
        return normal_color;
    }     
    
	
};

MTS_IMPLEMENT_CLASS_S(NormIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(NormIntegrator, "Integrator that shows the normals of the surfaces");

MTS_NAMESPACE_END
