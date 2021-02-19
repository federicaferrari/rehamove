#ifndef INTERFACE_H
#define INTERFACE_H
#include <QtCore>
#include "stimulation.h"
#include "simulation_torque.h"




class interface :public QThread //it has a public function called run
{
public:
    interface();
    void run(); //when you start thread this function will be called

    bool mStop;
    bool mStart;
    int mode;
    int a=0;

    stimulation mStimulation;
    simulation_torque simu;



};

#endif // INTERFACE_H
