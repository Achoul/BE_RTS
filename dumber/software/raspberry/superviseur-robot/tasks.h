/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TASKS_H__
#define __TASKS_H__


#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    
    struct camera_struct_t{
        Camera camera = Camera(sm,10); //added (10 fps for 1 frame every 100ms)
        bool cameraEnabled = false; //for task draw on picture
    }camera_struct;
    
    Img * currentImage; //image courrante (par période de sendImage), permet le traitement de la même image par plusieur tâche avant envoie au moniteur, moyénant synchronisation.
    
    struct struct_arena_t{ //structure contenant et l'arène et un flag
        Arena thunderdome; //nom de l'arène
        bool areneOK = false; //bool pour arène valide et existante (pour les tâches qui ne sont pas calibarene)
    }struct_arena;

    int robotStarted = 0; //flag pour savoir si le robot est en branle
    bool validArene = false; //prend la valeur de la réponse de l'utilisateur (arène choisie ou non) != areneOK 
    //validArene fait un peu doublon avec areneOK, à optimiser.
    bool positionRobotEnabled; //flag pour signaler que la position est active ou non.
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    RT_TASK th_battery; //s'occupe de tranmettre périodiquement le niveau de la batterie (T=500ms)
    RT_TASK th_sendImage; //s'occupe de tranmettre périodiquement le flux video au moniteur (T=100ms) + dessin de l'arène
    RT_TASK th_startCamera; //démarre la camera
    RT_TASK th_stopCamera; //arrête la camera
    RT_TASK th_calibrationArena; //calibre l'arène
    RT_TASK th_robotPosition; //calcule la position du robot
    RT_TASK th_connexionToRobotLost; //gère la perte de connexion avec le robot
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_camera;
    RT_MUTEX mutex_arena;
    RT_MUTEX mutex_currentImage;
    RT_MUTEX mutex_validArene;
    RT_MUTEX mutex_positionRobotEnabled;
    //protège respectivement ce qu'il y a dans leur nom

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    RT_SEM sem_startCamera; //added for when the camera needs to be started
    RT_SEM sem_stopCamera; //added for when the camera is stoped
    RT_SEM sem_calibTheThunderdome; //for when the calibration is needed
    RT_SEM sem_choosingArena; //for the selection of the arena
    RT_SEM sem_fluxOn; //for stopping camera flux
    RT_SEM sem_positionRobotOn; //for displaying the robot position
    RT_SEM sem_positionTreatment; //for attente de la fin du traitement de l'image dans robot position
    RT_SEM sem_computePos; //for rentrer dans le code qui dessine la position
    //petit mélange anglais-français pour faire varié les plaisirs

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);
    
    void BatteryTask();
    void StartCameraTask();
    void SendImageTask();
    void StopCameraTask();
    void CalibrationArenaTask();
    void RobotPositionTask();
    void ConnexionToRobotLostTask();

};

#endif // __TASKS_H__ 

