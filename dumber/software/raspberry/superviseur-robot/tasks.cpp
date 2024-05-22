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

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 1
#define PRIORITY_TOPENCOMROBOT 3
#define PRIORITY_TMOVE 8
#define PRIORITY_TSENDTOMON 5
#define PRIORITY_TRECEIVEFROMMON 4

#define PRIORITY_TSTARTROBOT 10
#define PRIORITY_TSTARTCAMERA 6
#define PRIORITY_TSENDIMAGE 11
#define PRIORITY_TSTOPCAMERA 9
#define PRIORITY_TBATTERY 13
#define PRIORITY_TCALIBARENA 7
#define PRIORITY_TROBOTPOSITION 6
#define PRIORITY_TCONNEXIONTOROBOTLOST 2

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_currentImage, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_validArene, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_positionRobotEnabled, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_calibTheThunderdome, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_choosingArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_fluxOn, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_positionRobotOn, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_computePos, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TSTARTCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImage, "th_sendImage", 0, PRIORITY_TSENDIMAGE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopCamera, "th_stopCamera", 0, PRIORITY_TSTOPCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_calibrationArena, "th_calibrationArena", 0, PRIORITY_TCALIBARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_robotPosition, "th_robotPosition", 0, PRIORITY_TROBOTPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_connexionToRobotLost, "th_connexionToRobotLost", 0, PRIORITY_TROBOTPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startCamera, (void(*)(void*)) & Tasks::StartCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImage, (void(*)(void*)) & Tasks::SendImageTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopCamera, (void(*)(void*)) & Tasks::StopCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_calibrationArena, (void(*)(void*)) & Tasks::CalibrationArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_robotPosition, (void(*)(void*)) & Tasks::RobotPositionTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_connexionToRobotLost, (void(*)(void*)) & Tasks::ConnexionToRobotLostTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) { 
            rt_sem_v(&sem_startCamera); //Put semaphore to start the camera to 1 when the message is received
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) { 
            rt_sem_v(&sem_stopCamera); //Put semaphore to start the camera to 1 when the message is received
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_calibTheThunderdome);
            
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            rt_mutex_acquire(&mutex_validArene, TM_INFINITE);//met validArene a 1 et respectivement a 0
            validArene = true;
            rt_mutex_release(&mutex_validArene);
            rt_sem_v(&sem_choosingArena);
            
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_validArene, TM_INFINITE);//met validArene a 1 et respectivement a 0
            validArene = false;
            rt_mutex_release(&mutex_validArene);
            rt_sem_v(&sem_choosingArena);  
            
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_mutex_acquire(&mutex_positionRobotEnabled, TM_INFINITE);
            positionRobotEnabled = true;
            rt_mutex_release(&mutex_positionRobotEnabled);
            rt_sem_v(&sem_positionRobotOn);

        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_mutex_acquire(&mutex_positionRobotEnabled, TM_INFINITE);
            positionRobotEnabled = false;
            rt_mutex_release(&mutex_positionRobotEnabled);
            rt_sem_p(&sem_positionRobotOn, TM_INFINITE);

            
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE); //for the 7 constraints just release this semaphore in task "connection to supervisor".
        cout << "Start robot without watchdog ("; 
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
    
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
}

void Tasks::BatteryTask(){
    
    int rs;
    MessageBattery * message_batt;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   

    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery update";
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {        
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            message_batt = (MessageBattery *)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);

            WriteInQueue(&q_messageToMon, message_batt);        }
        cout << endl << flush;
    }
}

void Tasks::StartCameraTask(){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    
     Message * message_open_camera;
    
    while(1){
        rt_sem_p(&sem_startCamera,TM_INFINITE);
        cout << "Starting da camera";
        cout << endl;
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        int status = camera_struct.camera.Open();
        camera_struct.cameraEnabled=true;
        rt_mutex_release(&mutex_camera);  
       
        
        if (status == 0) {
            message_open_camera = new Message(MESSAGE_ANSWER_NACK);
            cout << "Your camera is not lauched, you failed";
        } else {
            message_open_camera = new Message(MESSAGE_ANSWER_ACK);
            rt_sem_v(&sem_fluxOn);
            
            cout << "The camera has been open successfully man";
        }
        WriteInQueue(&q_messageToMon, message_open_camera);
        cout << endl << flush;
    }
    
}

void Tasks::SendImageTask(){
    
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
   
    MessageImg * message_image;
    bool cameraOn;
    bool positionEnabled_dummy;

    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    
    
    
    while (1) {
        rt_sem_p(&sem_fluxOn,TM_INFINITE);
 
        rt_task_wait_period(NULL); 
        
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        cameraOn = camera_struct.cameraEnabled;
        rt_mutex_release(&mutex_camera);
        
        if(cameraOn){
            rt_mutex_acquire(&mutex_currentImage, TM_INFINITE); //attention c'est risque
            currentImage = new Img(camera_struct.camera.Grab());
            rt_mutex_release(&mutex_currentImage);    
        }

        
        if(cameraOn){
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            if(struct_arena.areneOK){
                rt_mutex_acquire(&mutex_currentImage, TM_INFINITE);
                currentImage->DrawArena(struct_arena.thunderdome);
                rt_mutex_release(&mutex_currentImage);
                cout << "drawing arena" << endl << flush;
            }
            else{
               cout << "not drawing arena" << endl << flush; 
            }
            rt_mutex_release(&mutex_arena);
            
            rt_mutex_acquire(&mutex_positionRobotEnabled, TM_INFINITE);
            positionEnabled_dummy = positionRobotEnabled;
            rt_mutex_release(&mutex_positionRobotEnabled);
            
            if(positionEnabled_dummy){
                rt_sem_v(&sem_computePos);
                rt_sem_p(&sem_positionTreatment,TM_INFINITE); //attend que la tâche qui dessine la position est finie
                cout << "drawing position finished" << endl << flush; 
            }
            
            

            rt_mutex_acquire(&mutex_currentImage, TM_INFINITE);
            message_image = new MessageImg(MESSAGE_CAM_IMAGE, currentImage);
            rt_mutex_release(&mutex_currentImage);


            WriteInQueue(&q_messageToMon, message_image);// a voir si on laisse l'envoie dans le mutex
        }

        cout << "Taking a picture man" << endl << flush;
        rt_sem_v(&sem_fluxOn);
    }
}
    




void Tasks::StopCameraTask(){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    
    while(1){
        
        rt_sem_p(&sem_stopCamera,TM_INFINITE);
        
        rt_sem_p(&sem_fluxOn, TM_INFINITE);
        
        cout << "Stoping my big bad camera";
        cout << endl;
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera_struct.camera.Close();
        camera_struct.cameraEnabled=false;
        rt_mutex_release(&mutex_camera);  
        

    }
}

void Tasks::CalibrationArenaTask(){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    //struct_arena * struct_arena_ptr;
    Arena * ArenaFinded;
    Img * imageArene;
    MessageImg * message_image;
    Message * Msg;
    bool validArene_dummy;
    
    while(1){
        
        rt_sem_p(&sem_calibTheThunderdome,TM_INFINITE);
        
        rt_sem_p(&sem_fluxOn, TM_INFINITE); //stop le flux incéssent de prise d'image a 10 fps par secondes
        
        cout << "entree dans calib" << endl << flush;
        //choppe un image pour la validation de l'arene
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        imageArene = new Img(camera_struct.camera.Grab());
        rt_mutex_release(&mutex_camera);
        

        
        //cherche l'arene
        ArenaFinded = new Arena(imageArene->SearchArena());
        
        cout << "arene cherché" << endl;
        //Si pas d'arene, instancie l'arene
        if(ArenaFinded->IsEmpty()){
            cout << "Arena not finded" << endl;
            Msg = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, Msg);
            
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            struct_arena.areneOK = false;
            rt_mutex_release(&mutex_arena);
            
            
        }
        else{
            cout << "Arena finded" << endl;
            //dessine l'arene pour l'utilisateur
            imageArene->DrawArena(*ArenaFinded);
            message_image = new MessageImg(MESSAGE_CAM_IMAGE, imageArene);
            WriteInQueue(&q_messageToMon, message_image);
        
            rt_sem_p(&sem_choosingArena, TM_INFINITE); // attend le choix de l'arene
            
            rt_mutex_acquire(&mutex_validArene, TM_INFINITE);
            validArene_dummy = validArene;
            rt_mutex_release(&mutex_validArene);
            
            if(validArene_dummy){
                cout << "Arena validated" << endl << flush;
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                struct_arena.thunderdome = *ArenaFinded;
                rt_mutex_release(&mutex_arena);
                
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                struct_arena.areneOK = true;
                rt_mutex_release(&mutex_arena);

            }
            else{
                cout << "Arena not validated" << endl << flush;
                
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                struct_arena.areneOK = false;
                rt_mutex_release(&mutex_arena);
            }
        }  
    cout << "flux on" << endl << flush;
    rt_sem_v(&sem_fluxOn); //start the camera again / image flux
        
    }
}

void Tasks::RobotPositionTask(){
    
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    bool positionEnabled_dummy;
    std::list<Position> robotList;
    struct_arena_t struct_arena_dummy;
    camera_struct_t camera_struct_dummy;
    MessagePosition * Msg2Send;
    Position position_dummy;
    
    
    
    while(1){
        
        rt_sem_p(&sem_positionRobotOn, TM_INFINITE);
        rt_sem_p(&sem_computePos, TM_INFINITE);
        cout << "Drawing robots" <<endl << flush;
        
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        struct_arena_dummy = struct_arena; //pour proteger struct_arena (on copie totalement donc pas de pointeur), c'est trop lourd peut-être ?
        rt_mutex_release(&mutex_arena);
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera_struct_dummy = camera_struct; //pour proteger camera enanbled, important ici sinon ça dessine sur des image qui n'existent pas ==> erreur
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_positionRobotEnabled, TM_INFINITE);
        positionEnabled_dummy = positionRobotEnabled;
        rt_mutex_release(&mutex_positionRobotEnabled);
        
        
        if(positionEnabled_dummy && camera_struct_dummy.cameraEnabled){ //on si la position des robots est enabled
            rt_mutex_acquire(&mutex_currentImage, TM_INFINITE);
            robotList = currentImage->SearchRobot(struct_arena_dummy.thunderdome);
            rt_mutex_release(&mutex_currentImage);
            
                if(!(robotList.empty())){
                    rt_mutex_acquire(&mutex_currentImage, TM_INFINITE);
                    int robotNum = currentImage->DrawAllRobots(robotList);
                    rt_mutex_release(&mutex_currentImage);
                    cout << "drawing"<<endl << flush;
                    
                    
                    cout << "Number of robots drawn : " << robotNum <<endl << flush;
                    
                        Msg2Send = new MessagePosition(MESSAGE_CAM_POSITION, robotList.front());
                        WriteInQueue(&q_messageToMon, Msg2Send);   
                    
                }
                else{
                    cout << "No robot in sight"<<endl << flush;
                    Msg2Send = new MessagePosition(MESSAGE_CAM_POSITION, position_dummy);
                    WriteInQueue(&q_messageToMon, Msg2Send); 
                }
            
            
        }
        else{
            
        }
        rt_sem_v(&sem_positionTreatment);
        rt_sem_v(&sem_positionRobotOn);
        
    }

}

void Tasks::ConnexionToRobotLostTask(){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    int pingSuccess;
    int unsuccessCounter = 0;
    int rs;
    Message * answer;
    
    rt_task_set_periodic(NULL, TM_NOW, 1000000000); //1s period
    
    while(1){
        rt_task_wait_period(NULL); 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if(rs ==1){
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(new Message(MESSAGE_ROBOT_PING)); //envoie un ping au robot
            rt_mutex_release(&mutex_robot);
            
            if (!(answer->CompareID(MESSAGE_ANSWER_ACK))){
                unsuccessCounter++; //on incrémente le compteur d'erreur
                cout << "connection lost "<< unsuccessCounter << " times in a row"<<endl << flush;
            }
            else{
                unsuccessCounter = 0;
                cout << "connection good "<<endl << flush;
            }
            
            if(unsuccessCounter >= 3){
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ROBOT_ERROR));
                rt_sem_v(&sem_openComRobot);
            }
            
        }
        
    }
    
}

