#include <iostream>
using namespace std;
#include <cmath>
#define _USE_MATH_DEFINES
#include <array>


//takes input [0, 1] for the left and right joystick, output Ux and Theta.dot Z, witch go into a twist file for the motor drivers
array<float, 2> motionconverter(float gauche, float droite) { 
    
    
    float b = 0.7; //base width(m) (TODO YAML.FILE)
    float r = 0.1; //radius of the wheel(m) (TODO YAML.FILE)
    int gain = 10;//gain de la fonction to ponder speed (TODO YAML.FILE)
    
    
    array<float, 2> X; //vecteur X 



    X[0] = gain*r/2*(gauche + droite); // calcul matriciel, [U] = r*[J]*[y] , donne Ux
    X[1] = gain*r/b*(droite-gauche); // calcul matriciel, [U] = r*[J]*[y] , donne thetadot Z



    return X;

}
