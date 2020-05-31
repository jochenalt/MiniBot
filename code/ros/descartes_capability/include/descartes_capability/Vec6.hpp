#ifndef VEC6_HPP
#define VEC6_HPP

#include <cmath>

struct Vec6 {

    float x, y, z;
    float yaw, nick, roll;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    Vec6() { x = float(0); y = float(0); z = float(0); yaw = float(0); nick = float(0); roll = float(0);}

    Vec6(float x_, float y_, float z_, float yaw_, float nick_, float roll_) { x = x_; y = y_; z = z_;roll = roll_;nick= nick_;yaw = yaw_ ;}

    explicit Vec6(float v) { x = v; y = v; z = v; yaw = 0; nick = 0;roll = 0;}

    static Vec6 unit_x()    { return Vec6(float(1), float(0), float(0), float(0), float(0), float(0)); }
    static Vec6 unit_y()    { return Vec6(float(0), float(1), float(0), float(0), float(0), float(0)); }
    static Vec6 unit_z()    { return Vec6(float(0), float(0), float(1), float(0), float(0), float(0)); }
    static Vec6 unit_yaw()    { return Vec6(float(0), float(0), float(0), float(1), float(0), float(0)); }
    static Vec6 unit_nick()    { return Vec6(float(0), float(0), float(0), float(0), float(1), float(0)); }
    static Vec6 unit_roll()    { return Vec6(float(0), float(0), float(0), float(0), float(0), float(1)); }

    static Vec6 zero()      { return Vec6(float(0), float(0), float(0), float(0), float(0), float(0)); }
    static Vec6 unit_scale(){ return Vec6(float(1), float(1), float(1), float(0), float(0), float(0)); }


    void set(float x_, float y_, float z_, float yaw_, float nick_, float roll_) { x = x_; y = y_; z = z_; yaw = yaw_; nick= nick_; roll = roll_;}

    // -------------------------------------------------------------------------
    /// @name Overload operators
    // -------------------------------------------------------------------------

    // ----------
    // Additions
    // ----------


    Vec6 operator+(const Vec6 &v_) const {
        return Vec6(x+v_.x, y+v_.y, z+v_.z, yaw+v_.yaw, nick+v_.nick, roll+v_.roll);
    }

    Vec6& operator+= (const Vec6 &v_) {
        x += v_.x;
        y += v_.y;
        z += v_.z;
        yaw  += v_.yaw;
        nick += v_.nick;
        roll += v_.roll;

        return *this;
    }

    Vec6 operator+(float f_) const {
        return Vec6(x+f_, y+f_, z+f_, yaw,nick,roll);
    }

    /// lhs scalar cwise addition
    friend Vec6 operator+(const float d_, const Vec6& vec) {
        return Vec6(d_+vec.x, d_+vec.y, d_+vec.z, vec.yaw, vec.nick, vec.roll);
    }

    Vec6& operator+= (float f_) {
        x += f_;
        y += f_;
        z += f_;
        return *this;
    }

    // -------------
    // Substractions
    // -------------


    Vec6 operator-(const Vec6 &v_) const {
        return Vec6(x-v_.x, y-v_.y, z-v_.z, yaw - v_.yaw, nick - v_.nick, roll - v_.roll);
    }


    Vec6& operator-= (const Vec6& v_) {
        x -= v_.x;
        y -= v_.y;
        z -= v_.z;
        yaw -= v_.yaw;
        nick -= v_.nick;
        roll -= v_.roll;

        return *this;
    }

    /// opposite vector

    Vec6 operator-() const {
        return Vec6(-x, -y, -z, -yaw, -nick,-roll);
    }

    Vec6 operator-(float f_) const { return Vec6(x-f_, y-f_, z-f_, yaw, nick, roll); }

    /// lhs scalar cwise substraction
    friend Vec6 operator-(const float d_, const Vec6& vec) {
        return Vec6(d_-vec.x, d_-vec.y, d_-vec.z, 0-vec.yaw, 0-vec.nick, 0-vec.roll);
    }

    Vec6& operator-= (float f_) {
        x -= f_;
        y -= f_;
        z -= f_;
        return *this;
    }

    // -------------
    // Comparisons
    // -------------

    bool operator!= (const Vec6 &v_) const {
        return (x != v_.x) |  (y != v_.y) | (z != v_.z);
    }


    bool operator==(const Vec6& d_) const {
        return (x == d_.x) && (y == d_.y) && (z == d_.z);
    }

    /// @note no mathematical meaning but useful to use Vec3 in std::map
    bool operator< (const Vec6& v_) const
    {
        if( x != v_.x)
            return x < v_.x;
        else if( y != v_.y )
            return y < v_.y;
        else
            return z < v_.z;
    }

    // -------------
    // Divisions
    // -------------

    Vec6 operator/(const float d_) const {
        return Vec6(x/d_, y/d_, z/d_, yaw/d_, nick/d_, roll/d_);
    }

    Vec6& operator/=(const float d_) {
        x /= d_;
        y /= d_;
        z /= d_;
        return *this;
    }

    Vec6 operator/(const Vec6 &v_) const {
        return Vec6(x/v_.x, y/v_.y, z/v_.z, yaw/v_.yaw, nick/v_.nick, roll/v_.roll);
    }

    // ----------------
    // Multiplication
    // ----------------

    /// rhs scalar multiplication
    Vec6 operator*(const float d_) const {
        return Vec6(x*d_, y*d_, z*d_, yaw*d_, nick*d_, roll*d_);
    }

    /// lhs scalar multiplication
    friend Vec6 operator*(const float d_, const Vec6& vec) {
        return Vec6(d_*vec.x, d_*vec.y, d_*vec.z, d_*vec.yaw, d_*vec.nick, d_*vec.roll);
    }

    Vec6& operator*=(const float d_) {
        x *= d_;
        y *= d_;
        z *= d_;
        yaw *= d_;
        nick *= d_;
        roll *= d_;

        return *this;
    }

    Vec6 operator*(const Vec6 &v_) const {
        return Vec6(x*v_.x, y*v_.y, z*v_.z, yaw*v_.yaw, nick*v_.nick, roll*v_.roll);
    }

    Vec6& operator*=(const Vec6& d_) {
        x *= d_.x;
        y *= d_.y;
        z *= d_.z;
        yaw *= d_.yaw;
        nick*= d_.roll;
        nick*= d_.roll;

        return *this;
    }

    // -------------------------------------------------------------------------
    /// @name Operators on vector
    // -------------------------------------------------------------------------

    /// cross product
    Vec6 cross(const Vec6& v_) const {
        return Vec6(y*v_.z-z*v_.y, z*v_.x-x*v_.z, x*v_.y-y*v_.x, yaw*v_.yaw, nick*v_.nick, roll*v_.roll);
    }

    /// dot product
    float dot(const Vec6& v_) const {
        return x*v_.x+y*v_.y+z*v_.z;
    }

    /// Compute the cotangente (i.e. 1./tan) between 'this' and v_
    float cotan(const Vec6& v_) const {
        // cot(alpha ) = dot(v1, v2) / ||cross(v1, v2)||
        // = ||v1||*||v2||*cos( angle(v1, v2) ) / ||v1||*||v2|| * sin( angle(v1, v2) )
        // = cos( angle(v1, v2) ) / sin( angle(v1, v2) )
        // = 1 / tan( angle(v1, v2) )
        // = cot( angle(v1, v2) ) = cot( alpha )
        return (this->dot(v_)) / (this->cross(v_)).norm();
    }

    float norm_squared() const {
        return dot(*this);
    }

    Vec6 normalized() const {
        return (*this) * (float(1)/std::sqrt(norm_squared()));
    }

    float normalize() {
        float l = std::sqrt(norm_squared());
        float f = float(1) / l;
        x *= f;
        y *= f;
        z *= f;
        return l;
    }

    float safe_normalize(const float eps = 1e-10) {
        float l = std::sqrt(norm_squared());
        if(l > eps){
            float f = float(1) / l;
            x *= f;
            y *= f;
            z *= f;
            return l;
        } else {
            x = float(1);
            y = float(0);
            z = float(0);
            return float(0);
        }
    }

    float norm() const {
        return std::sqrt(norm_squared());
    }

    // -------------------------------------------------------------------------
    /// @name Accessors
    // -------------------------------------------------------------------------

     const float& operator[](int i) const{
        return ((const float*)this)[i];
    }

     float& operator[](int i){
        return ((float*)this)[i];
    }

    /// Conversion returns the memory address of the vector.
    /// Very convenient to pass a Vec pointer as a parameter to OpenGL:
    /// @code
    /// Vec3 pos, normal;
    /// glNormal3fv(normal);
    /// glVertex3fv(pos);
    /// @endcode
     operator const float*() const { return &x; }

    /// Conversion returns the memory address of the vector. (Non const version)
     operator float*() { return &x; }

};

#endif // VEC3_HPP
