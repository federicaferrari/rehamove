
// HEADERS
#include "smpt_ll_client.h"
#include "smpt_client.h"
#include "smpt_ml_client.h"
#include "smpt_messages.h"
#include "smpt_packet_number_generator.h"
#include "smpt_ll_packet_validity.h"



#include <iostream>
#include <stdint.h>

#include <ostream>
#include <QtDebug>
#include <thread>
#include <chrono>

#include <string>
#include <stdio.h>
using namespace std;

#ifdef __unix__
# include <unistd.h>
#elif defined _WIN32
# include <windows.h>
#define sleep(x) Sleep(1000 * (x))
#endif


#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include <unistd.h>
#include <pthread.h>


#include <QtCore/QCoreApplication>

#include <termios.h>
#include <fcntl.h>


int usleep(useconds_t usec);



// VARIABLES

bool check_open;
int digit=0;
bool check_close;
bool check_sent;
bool check_received;
bool check_status;
bool check_stop;
int i;
int j;
bool check_send_ml_init;
bool check_send_ml_update;
bool check_data;

int number_of_points; // 1-16
int ramp=0; // 0-15
float period=0; // [0,5–16383] ms ([<0.1-2000] Hz)

static void init_stimulation(Smpt_device *const device);
static void single_pulse(Smpt_device *const device);
static void stop_stimulation(Smpt_device *const device);

static void start_timing(void);
static int  stop_timing(void);

struct timeval start_, stop_;
long u1_ = 0, u2_ = 0;

float corrente[16];
int durata[16];

float current_ll[16];
int duration_ll[16];

int durata_session;
int durata_stimolo;
int durata_pausa;
int numero_eventi;
int tempo_trascorso;

#define MAX_CORRENTE 10
#define MAX_DURATA 400

int flag=0;

// FUNCTIONS

static void fill_ml_init(Smpt_device * const device, Smpt_ml_init *const ml_init);
static void fill_ml_update(Smpt_device * const device, Smpt_ml_update *const ml_update);
static void fill_ml_get_current_data(Smpt_device * const device, Smpt_ml_get_current_data *const ml_get_current_data);

//void myturn();
//void yourturn();

int main()
{

    // Modify with the name of your port
    const char *port_name= "/dev/ttyUSB0";

    // Open serial port
    Smpt_device device= {0};
    check_open= smpt_open_serial_port(&device, port_name);
    // uncomment to check if the port is open
    //qDebug() << "la porta è aperta? " << check_open;

    cout << "1: close - 2: low level stimulation - 3 mid level stimulation - 4 sp ll adv stimulation ";
    cin >> digit;

    if (digit==1){

        check_close=smpt_close_serial_port(&device);
        // uncomment to check if the port is closed
        //qDebug() << "la porta è chiusa? " << check_close;
    }

    else if(digit==2){

        printf("LOW LEVEL STIMULATION \n");

        //PARAMETERS SETTINGS
        // number of points [1-16]
        //ramp [0-15]
        //period (1/freq) [0.5 16383] ms
        //current [-150 150] mA
        //pulse duration [0 4095] µs

        printf("Inserire parametri \n");
        cout << "Number of points:";
        cin >> number_of_points;
        if(number_of_points<1 || number_of_points>16){
            printf("Value not permitted");
            return 0;
        }


        uint8_t packet_number = 0;  /* The packet_number can be used for debugging purposes */
        Smpt_ll_init ll_init = {0};       /* Struct for ll_init command */
        Smpt_ll_channel_config ll_channel_config = {0};   /* Struct for ll_channel_config command */

        /* Clear ll_init struct and set the data */
        smpt_clear_ll_init(&ll_init);
        ll_init.packet_number = packet_number;

        /* Send the ll_init command to stimulation unit */
        smpt_send_ll_init(&device, &ll_init);

        packet_number++;

        /* Set the data */
        ll_channel_config.enable_stimulation = true;
        ll_channel_config.channel = Smpt_Channel_Red;
        ll_channel_config.number_of_points = number_of_points;
        ll_channel_config.packet_number = packet_number;

        for (j=0; j<number_of_points; j++){

            cout << "inserire valori corrente per ciascun punto (se è prevista una pausa inserire 0):";
            cin >> current_ll[j];
            if(current_ll[j]<-150 || current_ll[j]>150){
                printf("Value not permitted");
                return 0;
            }
        }


        for (j=0; j<number_of_points; j++){

            cout << "inserire i valori di durata di ciascun punto (in micros)";
            cin >> duration_ll[j];
            if(duration_ll[j]<0 || duration_ll[j]>4095){
                printf("Value not permitted");
                return 0;
            }
        }

        for (j=0; j<number_of_points; j++){

            ll_channel_config.points[j].current =  current_ll[j];
            ll_channel_config.points[0].time = duration_ll[j];

        }

        int numero=0;
        cout << "premi un tasto per continuare: \n";
        cin.get();


            /* Send the ll_channel_list command to the stimulation unit */


            //                 check_sent=smpt_send_ll_channel_config(&device, &ll_channel_config);
            //                 qDebug() << "check sent " << check_sent;
            //                 sleep(2);

            //                 check_sent=smpt_send_ll_channel_config(&device, &ll_channel_config);
            //                 qDebug() << "check sent " << check_sent;
            //                 sleep(2);

            //                 check_sent=smpt_send_ll_channel_config(&device, &ll_channel_config);
            //                 qDebug() << "check sent " << check_sent;
            //                 sleep(2);

            //                 check_received=smpt_new_packet_received(&device);
            //                 qDebug() << "check received " << check_received;
            //                 check_status=smpt_send_get_stim_status(&device, packet_number);
            //                 qDebug() << "check status " << check_status;



            for (i=0; i<67; i++){

                check_sent=smpt_send_ll_channel_config(&device, &ll_channel_config);
                qDebug() << "check sent " << check_sent;
                usleep(30*1000);
            }



        packet_number++;


    }

//    //  Send the ll_stop command to the stimulation unit
//    check_stop=smpt_send_ll_stop(&device, packet_number);
//    qDebug() << "check stop " << check_stop;

//    check_close= smpt_close_serial_port(&device);
//    qDebug() << "la porta è chiusa? " << check_close;



    else if (digit==3){

        printf("Stimolazione MID LEVEL!!\n");

        // inserire parametri per la session
        cout << "durata della session: ";
        cin >> durata_session;
        cout << "durata stimolo: ";
        cin >> durata_stimolo;
        cout << "durata pausa: ";
        cin >> durata_pausa;

        numero_eventi = durata_session/(durata_pausa+durata_stimolo);
        printf("numero eventi: %d \n", numero_eventi);


        Smpt_ml_init ml_init = {0};           /* Struct for ml_init command */
        fill_ml_init(&device, &ml_init);
        check_send_ml_init=smpt_send_ml_init(&device, &ml_init); /* Send the ml_init command to the stimulation unit */
        qDebug() << "check send ml init" << check_send_ml_init;

        Smpt_ml_update ml_update = {0};       /* Struct for ml_update command */
       //fill_ml_update(&device, &ml_update);


        //PARAMETERS SETTINGS
        // number of points [1-16]
        //ramp [0-15]
        //period (1/freq) [0.5 16383] ms
        //current [-150 150] mA
        //pulse duration [0 4095] µs

        printf("Inserire parametri \n");
        cout << "Number of points:";
        cin >> number_of_points;
        cout << "Ramp - Number of linear increasing lower current pulse pattern until the full current is reached: ";
        cin >> ramp;
        cout << "Period: ";
        cin >> period;


        for (j=0; j<number_of_points; j++){

            cout << "inserire valori corrente per ciascun punto (se è prevista una pausa inserire 0): ";
            cin >> corrente[j];
        }

        for (j=0; j<number_of_points; j++){

            cout << "inserire i valori di durata di ciascun punto (in micros): ";
            cin >> durata[j];
        }

        /* Clear ml_update and set the data */
        smpt_clear_ml_update(&ml_update);
        ml_update.enable_channel[Smpt_Channel_Red] = true;
        ml_update.packet_number = smpt_packet_number_generator_next(&device);

        ml_update.channel_config[Smpt_Channel_Red].number_of_points = number_of_points;
        ml_update.channel_config[Smpt_Channel_Red].ramp = ramp;
        ml_update.channel_config[Smpt_Channel_Red].period = period;


        for(j=0; j<number_of_points; j++)
        {
           ml_update.channel_config[Smpt_Channel_Red].points[j].current = corrente[j];
           ml_update.channel_config[Smpt_Channel_Red].points[j].time = durata[j];

        }

        check_send_ml_update=smpt_send_ml_update(&device, &ml_update);
        qDebug() << "check send ml update" << check_send_ml_update;


        Smpt_ml_get_current_data ml_get_current_data = {0};
        fill_ml_get_current_data(&device, &ml_get_current_data);
        check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
        qDebug() << "check data" << check_data;

        //usleep(300*1000); //se voglio stimolare nel mentre
        //sleep(1);
        sleep(2); //se voglio stimolare in coda

        // cambio parametri e stimolo in coda

        for(j=0; j<number_of_points; j++){

            if(corrente[j]>0){
                corrente[j]=corrente[j]-5;}
            else if(corrente[j]<0){
                corrente[j]=corrente[j]+5;
            }

        }

        for(j=0; j<number_of_points; j++)
        {
           ml_update.channel_config[Smpt_Channel_Red].points[j].current = corrente[j];
           ml_update.channel_config[Smpt_Channel_Red].points[j].time = durata[j];

        }


        smpt_send_ml_update(&device, &ml_update);

        //commentare queste tre righe e lasciare send_ml_update per update nel mentre
        fill_ml_get_current_data(&device, &ml_get_current_data);
        check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
        qDebug() << "check data" << check_data;

//        if(flag==1){


//            for(j=0; j<number_of_points; j++){

//                if(ml_update.channel_config[Smpt_Channel_Red].points[j].current>0){
//                    ml_update.channel_config[Smpt_Channel_Red].points[j].current=ml_update.channel_config[Smpt_Channel_Red].points[j].current-2;}
//                else if(ml_update.channel_config[Smpt_Channel_Red].points[j].current<0){
//                    ml_update.channel_config[Smpt_Channel_Red].points[j].current=ml_update.channel_config[Smpt_Channel_Red].points[j].current+2;}

//            }

//            smpt_send_ml_update(&device, &ml_update);
//            fill_ml_get_current_data(&device, &ml_get_current_data);
//            check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);


//        }


//        fill_ml_update(&device, &ml_update);
//        check_send_ml_update=smpt_send_ml_update(&device, &ml_update);
//        fill_ml_get_current_data(&device, &ml_get_current_data);
//        check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);



    }

    else if(digit ==4){

        printf("Stimolazione LOW LEVEL ADVANCED!!\n");

        init_stimulation(&device);
        single_pulse(&device);    /* Sends one bi-phasic impulse and checks the response */
        stop_stimulation(&device);
        smpt_close_serial_port(&device);

    }

    else if(digit==5)
    {

        Smpt_device device = {0};
            smpt_open_serial_port(&device, port_name);

            Smpt_ml_init ml_init = {0};           /* Struct for ml_init command */
            fill_ml_init(&device, &ml_init);
            smpt_send_ml_init(&device, &ml_init); /* Send the ml_init command to the stimulation unit */

            Smpt_ml_update ml_update = {0};       /* Struct for ml_update command */
            fill_ml_update(&device, &ml_update);
            smpt_send_ml_update(&device, &ml_update);

            Smpt_ml_get_current_data ml_get_current_data = {0};
            fill_ml_get_current_data(&device, &ml_get_current_data);
            smpt_send_ml_get_current_data(&device, &ml_get_current_data);

            smpt_send_ml_stop(&device, smpt_packet_number_generator_next(&device));

            smpt_close_serial_port(&device);

    }


    return 0;

}


//************************************************************************************************************************************************************************************************//
//************************************************************************************************************************************************************************************************//


// FUNCTIONS - LOW LEVEL ADVANCED STIMULATION

void init_stimulation(Smpt_device *const device)
{
    Smpt_ll_init ll_init = {0};  /* Struct for ll_init command */
    Smpt_ll_init_ack ll_init_ack = {0};  /* Struct for ll_init_ack response */
    Smpt_ack ack = {0};  /* Struct for general response */

    smpt_clear_ll_init(&ll_init);
    ll_init.packet_number = smpt_packet_number_generator_next(device);

    printf("SMPT init_stimulaton(): send Ll_init command ...\n");
    start_timing();
    smpt_send_ll_init(device, &ll_init);   /* Send the ll_init command to the stimulation unit */

    while (!smpt_new_packet_received(device)) { /* busy waits for Ll_init_ack response */}
    printf("SMPT init_stimulaton(): Ll_init_ack received, took %d ms...\n", stop_timing());

    smpt_clear_ack(&ack);
    smpt_last_ack(device, &ack);
    if (ack.command_number == Smpt_Cmd_Ll_Init_Ack)
    {
        smpt_get_ll_init_ack(device, &ll_init_ack);  /* Writes the received data into ll_init_ack */
        if ( ll_init_ack.result == Smpt_Result_Successful )
        {
            printf("SMPT init_stimulation(): Ll_init command was successful\n");
        }
        else
        {
            printf("SMPT init_stimulation(): Ll_init command failed! Return code: %d\n", (int)ll_init_ack.result);
        }
    }
    else
    {
        printf("SMPT init_stimulation(): Unexpected ack received! Command ID: %d\n", (int)ack.command_number);
    }
    printf("\n\n");
}


void single_pulse(Smpt_device *const device)
{
    Smpt_ll_channel_config ll_channel_config         = {0}; /* Struct for ll_channel_config command */
    Smpt_ll_channel_config_ack ll_channel_config_ack = {0}; /* Struct for the ll_channel_config_ack response */
    Smpt_ack ack = {0};                                     /* Struct for general response */

    printf("SMPT single_pulse(): send Puls_Config command ...\n");
    start_timing();

    /* Set the data */
    ll_channel_config.enable_stimulation = true;
    ll_channel_config.channel = Smpt_Channel_Red;  /* Use red channel */
    ll_channel_config.number_of_points = 3;        /* Set the number of points*/
    ll_channel_config.packet_number = smpt_packet_number_generator_next(device);

    /* Set the stimulation pulse */
    /* First point, current: 20 mA, positive, pulse width: 200 µs */
    ll_channel_config.points[0].current =  5;
    ll_channel_config.points[0].time    = 300;

    /* Second point, pause 100 µs */
    ll_channel_config.points[1].time = 100;

    /* Third point, current: -20 mA, negative, pulse width: 200 µs */
    ll_channel_config.points[2].current = -5;
    ll_channel_config.points[2].time    = 300;

    /* Send the ll_channel_list command to the stimulation unit */
    smpt_send_ll_channel_config(device, &ll_channel_config);

    while (!smpt_new_packet_received(device)) { /* busy waits for ll_stop_ack */ }
    /* Get start time for timing measurements */
    start_timing();

    smpt_clear_ack(&ack);
    smpt_last_ack(device, &ack);
    if (ack.command_number == Smpt_Cmd_Ll_Channel_Config_Ack)
    {
        smpt_get_ll_channel_config_ack(device, &ll_channel_config_ack);  /* Writes the received data into ll_channel_config_ack */
        /* Give Feedback about the initialisation */
        if ( ll_channel_config_ack.result == Smpt_Result_Successful )
        {
            printf("SMPT single_pulse_test: Puls_Config command was successful! (took %fms)\n", stop_timing());
        }
        else
        {
            printf("SMPT single_pulse_test: Puls_Config command failed! Return code: %d; Electrode Error Code: %d\n", (int)ll_channel_config_ack.result, (int)ll_channel_config_ack.electrode_error);
            stop_timing();
        }
    }
    else
    {
        printf("SMPT single_pulse_test: Unexpected ack received! Command ID: %d\n", (int)ack.command_number);
    }
    printf("\n\n");
}

void stop_stimulation(Smpt_device *const device)
{
    Smpt_ack ack = {0};

    printf("SMPT wait_for_ACK_test: send Stop command ...\n");
    start_timing();
    smpt_send_ll_stop(device, smpt_packet_number_generator_next(device));

    while (!smpt_new_packet_received(device)) { /* busy waits for Ll_stop_ack response */}

    smpt_clear_ack(&ack);
    smpt_last_ack(device, &ack);
    if (ack.command_number == Smpt_Cmd_Ll_Stop_Ack)
    {
        /* Ll_stop_ack has been received */
        printf("SMPT wait_for_ACK_test: Stop ack received, took %d ms...\n", stop_timing());
    }
    else
    {
        printf("SMPT wait_for_ACK_test: Unexpected ack received! Command ID: %d\n", (int)ack.command_number);
        stop_timing();
    }

    printf("\n\n");
}


void start_timing(void)
{
    /* Get start time for timing measurements */
    gettimeofday(&start_, NULL);
    u1_ = start_.tv_sec * 1000 + start_.tv_usec / 1000;
}

int stop_timing(void)
{
    /* Get start time for timing measurements */
    gettimeofday(&stop_, NULL);
    u2_ = stop_.tv_sec*1000 + stop_.tv_usec/1000;

    return (int)(u2_-u1_);
}



//************************************************************************************************************************************************************************************************//
//************************************************************************************************************************************************************************************************//

//FUNCTIONS - MID LEVEL STIMULATION


void fill_ml_init(Smpt_device *const device, Smpt_ml_init *const ml_init)
{
    /* Clear ml_init struct and set the data */
    smpt_clear_ml_init(ml_init);
    ml_init->packet_number = smpt_packet_number_generator_next(device);
}

void fill_ml_update(Smpt_device *const device, Smpt_ml_update *const ml_update)
{


    //PARAMETERS SETTINGS
    // number of points [1-16]
    //ramp [0-15]
    //period (1/freq) [0.5 16383] ms
    //current [-150 150] mA
    //pulse duration [0 4095] µs

    printf("Inserire parametri \n");
    cout << "Number of points:";
    cin >> number_of_points;
    cout << "Ramp - Number of linear increasing lower current pulse pattern until the full current is reached: ";
    cin >> ramp;
    cout << "Period: ";
    cin >> period;


    for (j=0; j<number_of_points; j++){

        cout << "inserire valori corrente per ciascun punto (se è prevista una pausa inserire 0): ";
        cin >> corrente[j];
    }

    for (j=0; j<number_of_points; j++){

        cout << "inserire i valori di durata di ciascun punto (in micros): ";
        cin >> durata[j];
    }

    /* Clear ml_update and set the data */
    smpt_clear_ml_update(ml_update);
    ml_update->enable_channel[Smpt_Channel_Blue] = true;
    ml_update->packet_number = smpt_packet_number_generator_next(device);

    ml_update->channel_config[Smpt_Channel_Blue].number_of_points = number_of_points;
    ml_update->channel_config[Smpt_Channel_Blue].ramp = ramp;
    ml_update->channel_config[Smpt_Channel_Blue].period = period;


    for(j=0; j<number_of_points; j++)
    {
       ml_update->channel_config[Smpt_Channel_Blue].points[j].current = corrente[j];
       ml_update->channel_config[Smpt_Channel_Blue].points[j].time = durata[j];

    }

}

void fill_ml_get_current_data(Smpt_device *const device, Smpt_ml_get_current_data *const ml_get_current_data)
{
    ml_get_current_data->packet_number = smpt_packet_number_generator_next(device);
    ml_get_current_data->data_selection[Smpt_Ml_Data_Stimulation] = true; /* get stimulation data */
}

