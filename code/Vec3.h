#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <cstdlib>
#include <iostream>

#define EPSI 1e-6
#define PI  3.1415926535897932384626
#define rand01() (rand() / double(RAND_MAX))

struct Vec3 {
	double x, y, z;
	Vec3() : x( 0 ), y( 0 ), z( 0 ) {}
	Vec3( double val ) : x( val ), y( val ), z( val ) {}
	Vec3( double x, double y, double z ) : x( x ), y( y ), z( z ) {}
	double length() const { return sqrt( length2() ); }
	double length2() const { return x * x + y * y + z * z; }
	double power() const { return (x + y + z) / 3; }

	Vec3 operator - () const { return Vec3( -x, -y, -z ); }
	Vec3 operator + ( const Vec3 &v ) const { return Vec3( x + v.x, y + v.y, z + v.z ); }
	Vec3 operator - ( const Vec3 &v ) const { return Vec3( x - v.x, y - v.y, z - v.z ); }
	Vec3 operator * ( const Vec3 &v ) const { return Vec3( x * v.x, y * v.y, z * v.z ); }
	Vec3 operator * ( double k ) const { return Vec3( x * k, y * k, z * k ); }
	Vec3 operator / ( double k ) const { return Vec3( x / k, y / k, z / k ); }
	Vec3 &operator += ( const Vec3 &v ) { return *this = *this + v; }
	Vec3 &operator -= ( const Vec3 &v ) { return *this = *this - v; }
	Vec3 &operator *= ( const Vec3 &v ) { return *this = *this * v; }
	Vec3 &operator *= ( double k ) { return *this = *this * k; }
	Vec3 &operator /= ( double k ) { return *this = *this / k; }
	bool operator == ( const Vec3 &v ) const { 
		return fabs( x - v.x ) < EPSI && fabs( y - v.y ) < EPSI && fabs( z - v.z ) < EPSI;
	}
	double &operator[] ( int i ) {
		if ( i == 0 ) return x; if ( i == 1 ) return y; if ( i == 2 ) return z;
		return x;
	}
	const double operator[] ( int i ) const {
		if ( i == 0 ) return x; if ( i == 1 ) return y; if ( i == 2 ) return z;
		return x;
	}

	Vec3 normalized() const {
		double len = length();
		double k = len ? 1 / len : 0;
		return Vec3( x * k, y * k, z * k );
	}
	Vec3 reflected( const Vec3 &N ) const {	// assert: |*this| == 1
		return *this - N * dot( *this, N ) * 2;
	} 
	Vec3 refracted( const Vec3 &N, double n ) const {	// assert: |*this| == 1
		double cosi = -dot( *this, N );
		double cosr2 = 1 - ( n * n ) * ( 1 - cosi * cosi );
		if ( cosr2 > EPSI ) return *this * n + N * ( n * cosi - sqrt( cosr2 ) );
		return this->reflected( N );
	}
	static Vec3 getRandomVector() {
		double x, y, z;
		do {
			x = 2 * rand01() - 1;
			y = 2 * rand01() - 1;
			z = 2 * rand01() - 1;
		} while ( x * x + y * y + z * z > 1 || x * x + y * y + z * z < EPSI );
		return Vec3( x, y, z ).normalized();
	}
	static Vec3 getCosRandomVector(const Vec3 &N) {
		double theta = acos(sqrt(rand01()));
	    double phi = 2 * PI * rand01();
	    Vec3 result = N.rotate(N.getVertical(), theta);
    	return result.rotate(N, phi).normalized();
	}

	Vec3 rotate(const Vec3 &axis, double angle) const {
	    if (fabs(angle) < EPSI) return Vec3(x, y, z);
	    Vec3 ret;
	    double cost = cos( angle );
	    double sint = sin( angle );
	    ret.x += x * ( axis.x * axis.x + ( 1 - axis.x * axis.x ) * cost );
	    ret.x += y * ( axis.x * axis.y * ( 1 - cost ) - axis.z * sint );
	    ret.x += z * ( axis.x * axis.z * ( 1 - cost ) + axis.y * sint );
	    ret.y += x * ( axis.y * axis.x * ( 1 - cost ) + axis.z * sint );
	    ret.y += y * ( axis.y * axis.y + ( 1 - axis.y * axis.y ) * cost );
	    ret.y += z * ( axis.y * axis.z * ( 1 - cost ) - axis.x * sint );
	    ret.z += x * ( axis.z * axis.x * ( 1 - cost ) - axis.y * sint );
	    ret.z += y * ( axis.z * axis.y * ( 1 - cost ) + axis.x * sint );
	    ret.z += z * ( axis.z * axis.z + ( 1 - axis.z * axis.z ) * cost );
	    return ret;
	}	
	Vec3 getVertical() const {
	    return (x == 0 && y == 0) ? Vec3(1, 0, 0) : Vec3(y, -x, 0).normalized();
	}
	friend std::ostream &operator << ( std::ostream &os, const Vec3 &v ) {
		os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
		return os;
	}
	friend double dot( const Vec3 &v1, const Vec3 &v2 ) {
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
	friend Vec3 cross( const Vec3 &v1, const Vec3 &v2 ) {
		return Vec3( v1.y * v2.z - v1.z * v2.y , v1.z * v2.x - v1.x * v2.z , v1.x * v2.y - v1.y * v2.x );
	}
	friend Vec3 operator* ( double k, const Vec3 &v ) { return v * k; }
};

#endif