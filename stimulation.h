#ifndef STIMULATION_H
#define STIMULATION_H
#include <QtCore>
#include "simulation_torque.h"




class stimulation :public QThread //it has a public function called run
{
public:
    stimulation();
    //void run(); //when you start thread this function will be called
    void start();
    void stop();
    void dosomething();
    void controllo();

    void torque();
    void pid();

    simulation_torque simu;



public:
    bool mStop;
    bool mUpdate;
    int digit;
    bool mStart;
    int mode;
    bool finish=false;
    double torque_imp;

private:
    int coppia=8;
    double val;
    double inc;
    int corr=7;

};

#endif // STIMULATION_H
