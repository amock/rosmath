#ifndef ROSMATH_EXCEPTIONS_H
#define ROSMATH_EXCEPTIONS_H

#include <exception>
#include <string>

namespace rosmath {

class TransformException : virtual public std::runtime_error 
{
protected:

    int error_number;               ///< Error number
    int error_offset;               ///< Error offset

public:

    /** Constructor (C++ STL string, int, int).
     *  @param msg The error message
     *  @param err_num Error number
     *  @param err_off Error offset
     */
    explicit 
    TransformException(
        const std::string& msg="Could not transform", 
        int err_num=0, 
        int err_off=0)
    :std::runtime_error(msg)
    {
        error_number = err_num;
        error_offset = err_off;
    }

    /** Destructor.
     *  Virtual to allow for subclassing.
     */
    virtual ~TransformException() throw () {}
    
    /** Returns error number.
     *  @return #error_number
     */
    virtual int getErrorNumber() const throw() {
        return error_number;
    }
    
    /**Returns error offset.
     * @return #error_offset
     */
    virtual int getErrorOffset() const throw() {
        return error_offset;
    }
};

} // namespace rosmath

#endif // ROSMATH_EXCEPTIONS_H