#ifndef MAF_INCLUDED
#define MAF_INCLUDED

#include "ofMain.h"

class ofxTktrMaf{

	public:
		
    
    
    
    float Maf_PI();
    float clamp(float v, float minVal, float maxVal);
    float step(float edge, float v);
    float smoothStep(float edge0, float edge1, float v);
    
    float deg2Rad(float degrees);
    float toRadians(float degrees);
    float rad2Deg(float radians);
    float toDegrees(float radians);
    
    float clamp01(float v);
    float mix(float x, float y, float a);
    float lerp(float x, float y, float a);
    float inverseMix(float a, float b, float v );
    float inverselerp(float a, float b, float v );
    float mixUnclamped(float x, float y, float a );
    float lerpUnclamped(float x, float y, float a );
    
    float fract(float v);
    float frac(float v);
    
    float isPowerOfTwo(float v);
    float closestPowerOfTwo(float v);
    float nextPowerOfTwo(float v);
    
    
    float mod(float a, float n);
    float deltaAngle(float a, float b);
    float deltaAngleDeg(float a, float b);
    float deltaAngleRad(float a, float b);
    float lerpAngle(float a, float b, float t);
    float lerpAngleDeg(float a, float b, float t);
    float lerpAngleRad(float a, float b, float t);
    
    
    float gammaToLinearSpace(float v);
    float linearToGammaSpace(float v);
    float map(float from1, float to1, float from2, float to2, float v);
    float scale(float from1, float to1, float from2, float to2, float v);
    
    float almostIdentity(float x, float m, float n);
    float impulse(float k, float x);
    float cubicPulse(float c, float w, float x);
    float expStep(float x, float k, float n);
    float parabola(float x, float k);
    float powerCurve(float x, float a, float b);
    
    
    ofVec3f latLonToCartesian(float lat, float lon);
    ofVec2f cartesianToLatLon(float x, float y, float z);
    float randomInRange(float min, float max);
    float norm(float v, float minVal, float maxVal);
    float hash(float n);
    float noise2d(float x, float y);
    
    
    float smoothMin(float a, float b, float k);
    float smoothMax(float a, float b, float k);
    float almost(float a, float b);
    
    
    
    
    
    
};

#endif
