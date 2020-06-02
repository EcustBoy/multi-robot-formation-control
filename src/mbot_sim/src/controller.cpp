#include <cmath>
#include <fstream>  
#include <sstream>

using namespace std;
double w1,v1;
double controller(double l_12,double theta_1,double theta_2,double phi_12,double v2,double w2,double l_12_d,double phi_12_d,double d,double k1,double k2){
    //l_12,theta_1,theta_2,phi_12,v_1,w_1: sensor value
    //l_12_d, phi_12_d: ref value
    //d,k1,k2: radius/controller params
    double pho_12;
    double gamma_12;

    gamma_12=theta_2+phi_12-theta_1;
    pho_12=(k1*(l_12_d-l_12)+v2*cos(phi_12))/cos(gamma_12);
    w1=cos(gamma_12)*(k2*l_12*(phi_12_d-phi_12)-v2*sin(phi_12)+l_12*w2+pho_12*sin(gamma_12))/d;
    v1=pho_12-d*w1*tan(gamma_12);

    //return w1,v1;


}