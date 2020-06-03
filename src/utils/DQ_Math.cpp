#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/utils/DQ_Constants.h>


namespace DQ_robotics
{

/**
 * @brief deg2rad
 * @param a
 * @return
 */
double deg2rad(const double& a)
{
    return (a)*2.0*pi/(360.0);
}

/**
 * @brief deg2rad
 * @param v
 * @return
 */
VectorXd deg2rad(const VectorXd& v)
{
    VectorXd ret(v);
    for(auto i=0;i<v.size();i++)
    {
        ret(i) = deg2rad(v(i));
    }
    return ret;
}

}
