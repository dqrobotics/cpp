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
    return (a)*pi/(180.0);
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

/**
 * @brief rad2deg
 * @param a
 * @return
 */
double rad2deg(const double& a)
{
    return (a)*180.0/(pi);
}

/**
 * @brief rad2deg
 * @param v
 * @return
 */
VectorXd rad2deg(const VectorXd& v)
{
    VectorXd ret(v);
    for(auto i=0;i<v.size();i++)
    {
        ret(i) = rad2deg(v(i));
    }
    return ret;
}

}
