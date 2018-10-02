#include "ofxTktrMaf.h"



float ofxTktrMaf::Maf_PI(){
    return 3.14159265359;
}

float ofxTktrMaf::clamp(float v, float minVal, float maxVal){
    return min(maxVal, max(minVal, v));
}

float ofxTktrMaf::step(float edge, float v){
    return ( v < edge ) ? 0 : 1;
}

float ofxTktrMaf::smoothStep(float edge0, float edge1, float v){
    float t = clamp(( v - edge0 ) / ( edge1 - edge0 ), 0.0, 1.0);
    return t * t * ( 3.0 - 2.0 * t );
}

float ofxTktrMaf::deg2Rad(float degrees){
    return degrees * Maf_PI() / 180;
}

float ofxTktrMaf::toRadians(float degrees){
    return deg2Rad(degrees);
}

float ofxTktrMaf::rad2Deg(float radians){
    return radians * 180 / Maf_PI();
}

float ofxTktrMaf::toDegrees(float radians){
    return rad2Deg(radians);
}


float ofxTktrMaf::clamp01(float v){
    return clamp( v, 0, 1 );
}


float ofxTktrMaf::mix(float x, float y, float a){
    if( a <= 0 ) return x;
    if( a >= 1 ) return y;
    return x + a * (y - x);
}


float ofxTktrMaf::lerp(float x, float y, float a){
    if( a <= 0 ) return x;
    if( a >= 1 ) return y;
    return x + a * (y - x);
}


float ofxTktrMaf::inverseMix(float a, float b, float v ){
    return ( v - a ) / ( b - a );
}

float ofxTktrMaf::inverselerp(float a, float b, float v ){
    return ( v - a ) / ( b - a );
}



float ofxTktrMaf::mixUnclamped(float x, float y, float a){
    if( a <= 0 ) return x;
    if( a >= 1 ) return y;
    return x + a * (y - x);
}

float ofxTktrMaf::lerpUnclamped(float x, float y, float a){
    if( a <= 0 ) return x;
    if( a >= 1 ) return y;
    return x + a * (y - x);
}



float ofxTktrMaf::fract(float v){
    return v - floor( v );
}


float ofxTktrMaf::frac(float v){
    return v - floor( v );
}


float ofxTktrMaf::isPowerOfTwo(float v){
    return ( ( ( v - 1) && v ) == 0 );
}


float ofxTktrMaf::closestPowerOfTwo(float v){
    return pow( 2, round( log( v ) / log( 2 ) ) );
}
float ofxTktrMaf::nextPowerOfTwo(float v){
    return pow( 2, ceil( log( v ) / log( 2 ) ) );
}


float ofxTktrMaf::mod(float a, float n){
    //return (a % n + n) % n;
    return a - floor(a/n) * n;
}
float ofxTktrMaf::deltaAngle(float a, float b){
    float d = mod( b - a, 360 );
    if( d > 180 ) d = abs( d - 360 );
    return d;
}
float ofxTktrMaf::deltaAngleDeg(float a, float b){
    float d = mod( b - a, 360 );
    if( d > 180 ) d = abs( d - 360 );
    return d;
}

float ofxTktrMaf::deltaAngleRad(float a, float b){
    return toRadians( deltaAngle( toDegrees( a ), toDegrees( b ) ) );
}

float ofxTktrMaf::lerpAngle(float a, float b, float t){
    float angle = deltaAngle( a, b );
    return mod( a + lerp( 0, angle, t ), 360 );
}
float ofxTktrMaf::lerpAngleDeg(float a, float b, float t){
    float angle = deltaAngle( a, b );
    return mod( a + lerp( 0, angle, t ), 360 );
}

float ofxTktrMaf::lerpAngleRad(float a, float b, float t){
    return toRadians( lerpAngleDeg( toDegrees( a ), toDegrees( b ), t ) );
}


float ofxTktrMaf::gammaToLinearSpace(float v){
    return pow( v, 2.2 );
}
float ofxTktrMaf::linearToGammaSpace(float v){
     return pow( v, 1 / 2.2 );
}
float ofxTktrMaf::map(float from1, float to1, float from2, float to2, float v){
    return from2 + ( v - from1 ) * ( to2 - from2 ) / ( to1 - from1 );
}
float ofxTktrMaf::scale(float from1, float to1, float from2, float to2, float v){
    return from2 + ( v - from1 ) * ( to2 - from2 ) / ( to1 - from1 );
}
float ofxTktrMaf::almostIdentity(float x, float m, float n){
    if( x > m ) return x;
    
    float a = 2 * n - m;
    float b = 2 * m - 3 * n;
    float t = x / m;
    
    return ( a * t + b) * t * t + n;
}
float ofxTktrMaf::impulse(float k, float x){
    float h = k * x;
    return h * exp( 1 - h );
}
float ofxTktrMaf::cubicPulse(float c, float w, float x){
    x = abs( x - c );
    if( x > w ) return 0;
    x /= w;
    return 1 - x * x * ( 3 - 2 * x );
}
float ofxTktrMaf::expStep(float x, float k, float n){
    return exp( -k * pow( x, n ) );
}
float ofxTktrMaf::parabola(float x, float k){
    return pow( 4 * x * ( 1 - x ), k );
}
float ofxTktrMaf::powerCurve(float x, float a, float b){
    float k = pow( a + b, a + b ) / ( pow( a, a ) * pow( b, b ) );
    return k * pow( x, a ) * pow( 1 - x, b );
}



ofVec3f ofxTktrMaf::latLonToCartesian(float lat, float lon){
    lon += 180;
    lat = clamp( lat, -85, 85 );
    float phi = toRadians( 90 - lat );
    float theta = toRadians( 180 - lon );
    float x = sin( phi ) * cos( theta );
    float y = cos( phi );
    float z = sin( phi ) * sin( theta );
    
    return ofVec3f(x, y, z);
}

ofVec2f ofxTktrMaf::cartesianToLatLon(float x, float y, float z){
    float n = sqrt( x * x + y * y + z * z );
    return ofVec2f(/*lat*/ asin( z / n ), /*lon*/ atan2( y, x ) );
}

float ofxTktrMaf::randomInRange(float min, float max){
    return min + ofRandom(0, 1) * ( max - min );
}

float ofxTktrMaf::norm(float v, float minVal, float maxVal){
    return ( v - minVal ) / ( maxVal - minVal );
}

float ofxTktrMaf::hash(float n){
    return fract( (1.0 + cos(n)) * 415.92653);
}

float ofxTktrMaf::noise2d(float x, float y){
    float xhash = hash( x * 37.0 );
    float yhash = hash( y * 57.0 );
    return fract( xhash + yhash );
}

float ofxTktrMaf::smoothMin(float a, float b, float k){
    float res = exp( -k*a ) + exp( -k*b );
    return - log( res )/ k;
}

float ofxTktrMaf::smoothMax(float a, float b, float k){
     return log( exp(a) + exp(b) )/k;
}

float ofxTktrMaf::almost(float a, float b){
    return ( abs( a - b ) < .0001 );
}


