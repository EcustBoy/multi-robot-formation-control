#include <cmath>
#include <fstream>  
#include <sstream>

using namespace std;
//double w_2,v_2;
double controller(double l_12,double theta_1,double theta_2,double phi_12,double v_1,double w_1,double l_12_d,double phi_12_d,double d,double k1,double k2){
    //l_12,theta_1,theta_2,phi_12,v_1,w_1: sensor value
    //l_12_d, phi_12_d: ref value
    //d,k1,k2: radius/controller params
    double w_2,v_2,pho_12;
    double gamma_12;

    gamma_12=theta_1+phi_12-theta_2;
    pho_12=(k1*(l_12_d-l_12)+v_1*cos(phi_12))/cos(gamma_12);
    w_2=cos(gamma_12)*(k2*l_12*(phi_12_d-phi_12)-v_1*sin(phi_12)+l_12*w_1+pho_12*sin(gamma_12))/d;
    v_2=pho_12-d*w_2*tan(gamma_12);

    return w_2,v_2;


}