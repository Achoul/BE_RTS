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
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 23
#define PRIORITY_TWATCHDOG 31

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
 * 5- Same behavior exist for ComMonitor::Write !
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
    if (err = rt_mutex_create(&mutex_capture, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       						  */
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
    if (err = rt_sem_create(&sem_getBattery, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startWithWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_capture, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_confirmArena, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_withWD, "th_withWD", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_acquireCamera, "th_acquireCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_arena, "th_arena", 0, PRIORITY_TCAMERA, 0)) {
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
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryLevel, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_withWD, (void(*)(void*)) & Tasks::StartWithWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::CameraOpen, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CameraClose, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_acquireCamera, (void(*)(void*)) & Tasks::CameraAcquire, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_arena, (void(*)(void*)) & Tasks::getArena, this)) {
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
void Tasks::ServerTask(void *arg) { // établir la connexion avec le moniteur
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
            WD = 0 ;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)) {
            rt_sem_v(&sem_getBattery);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            WD = 1 ;
            rt_sem_v(&sem_startWithWD);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_sem_v(&sem_openCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_sem_v(&sem_closeCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_arena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            isArenaConfirmed = true ;
            rt_sem_v(&sem_confirmArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            isArenaConfirmed = false ;
            rt_sem_v(&sem_confirmArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            positionRobot = true ;
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            positionRobot = false ;    
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
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        
        if (WD == 1){
            msgSend = robot.Write(robot.StartWithWD());
        }
        else {
            msgSend = robot.Write(robot.StartWithoutWD());
        }
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
}

/**
 * 
                            ADD OF THE ADDITIONNAL TASKS
 * 
 */

/**
 * Handle battery level
 */
void Tasks::BatteryLevel(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_getBattery, TM_INFINITE); // It gets the battery only when we specify it on the monitor
    
    /**************************************************************************************/
    /* The task BatteryLevel starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the battery task" << __PRETTY_FUNCTION__ << endl << flush;
    
    Message *msg ;
    bool isRobotStarted ;
    
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        if (isRobotStarted == 1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);

            cout << "Battery Level: " << msg->ToString() << endl << flush ;
            WriteInQueue(&q_messageToMon, msg);  // msg will be deleted by sendToMon
        }
    }
}

/**
 * Handle the start with WatchDog
 */
void Tasks::StartWithWD(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_startWithWD, TM_INFINITE); // It starts with WatchDog only when we specify it on the monitor
    
    /**************************************************************************************/
    /* The task Start_with_WD starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the StartWithWD task" << __PRETTY_FUNCTION__ << endl << flush;
    
    const int periodWD = 1000000000 ;
    Message *msg ;
    bool isRobotStarted ;
    
    rt_task_set_periodic(NULL, TM_NOW, periodWD);

    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        if (isRobotStarted == 1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);

            cout << " WatchDog : " << msg->ToString() << endl << flush ;
            WriteInQueue(&q_messageToMon, msg);  // msg will be deleted by sendToMon
        }
    }
}

/**
 * Handle the opening of the camera
 */
void Tasks::CameraOpen(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task CameraOpen starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the task" << __PRETTY_FUNCTION__ << endl << flush;
    
    cam = new Camera(sm, 10);
    Message *msg ;
    int isRobotStarted ;
    bool isCameraOpen ;
    
    while(1) {
        rt_sem_p(&sem_openCamera, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        if (isRobotStarted ==1) {
        
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            isCameraOpen = cam->IsOpen();
            rt_mutex_release(&mutex_camera);

            if (not isCameraOpen) {
                
                rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                isCameraOpen = cam->Open() ; // It returns if the camera has correctly been open
                rt_mutex_release(&mutex_camera);
                
                if (isCameraOpen) {
                    msg = new Message(MESSAGE_ANSWER_ACK);
                    cout << "Succeded To Open Camera"  << msg->ToString() <<endl ;
                    
                    rt_mutex_acquire(&mutex_capture,TM_INFINITE);
                    sendCapture = true ;
                    rt_mutex_release(&mutex_capture);
                    rt_sem_v(&sem_capture) ;
                    
                } else {
                    msg = new Message(MESSAGE_ANSWER_NACK);
                    cout << "Failed To Open Camera" << msg->ToString() << endl << flush;
                }
                WriteInQueue(&q_messageToMon, msg);  // msg will be deleted by sendToMon
            }
        }
    }
}

/**
 * Handle the closing of the camera
 */
void Tasks::CameraClose(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task CameraClose starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the task" << __PRETTY_FUNCTION__ << endl << flush;
    
//    cam = new Camera(sm, 10);
    int isRobotStarted ;
    bool isCameraOpen ;
    Message *msg ;
    
    while(1) {
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        if (isRobotStarted == 1) {
            
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            isCameraOpen = cam->IsOpen();
            rt_mutex_release(&mutex_camera);
            
            if (isCameraOpen) {
        
                rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                cam->Close(); // La primitive Close ne renvoie rien, donc on vérifiera que la fermeture a bien été effectuée proprement
                rt_mutex_release(&mutex_camera);
                
                // Est-ce que la caméra a bien été fermée ?
                rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                isCameraOpen = cam->IsOpen();
                rt_mutex_release(&mutex_camera);

                if (not isCameraOpen) {
                    msg = new Message(MESSAGE_ANSWER_ACK);
                    cout << "Succeded To Close Camera"  << endl ;
                    
                    rt_mutex_acquire(&mutex_capture,TM_INFINITE);
                    sendCapture = false ;
                    rt_mutex_release(&mutex_capture);
                }
                else {
                    msg = new Message(MESSAGE_ANSWER_NACK);
                    cout << "Failed To Close Camera" << msg->ToString() << endl << flush;
                }
                WriteInQueue(&q_messageToMon, msg);  // msg will be deleted by sendToMon
            }
        }
    }
}

/**
 * Handle the images acquisition by the camera
 */
void Tasks::CameraAcquire(void *arg) {
            
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_capture, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task CameraAcquire starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the task" << __PRETTY_FUNCTION__ << endl << flush;
    
    int isRobotStarted ;
    bool isCameraOpen ;
    bool iCanAcquire ;
    list<Position> positions ;
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    
    while(1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        isCameraOpen = cam->IsOpen();
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_capture,TM_INFINITE);
        iCanAcquire = sendCapture ;
        rt_mutex_release(&mutex_capture);
        

        if (isRobotStarted ==1 && isCameraOpen && iCanAcquire) {
                
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            Img * image = new Img(cam->Grab()) ;
            rt_mutex_release(&mutex_camera);
                
            rt_mutex_acquire(&mutex_arena,TM_INFINITE);
            Arena myArena = arena ;
            rt_mutex_release(&mutex_arena);

            if (not myArena.IsEmpty()) {
                image->DrawArena(myArena) ;
                if (positionRobot == 1) {
                    positions = image->SearchRobot(myArena) ;
                    image->DrawAllRobots(positions) ;
                }
            }
                       
            MessageImg *ImageToBeSend = new MessageImg(MESSAGE_CAM_IMAGE, image) ;
            rt_mutex_acquire(&mutex_monitor,TM_INFINITE);
            monitor.Write(ImageToBeSend) ;
            rt_mutex_release(&mutex_monitor);
            WriteInQueue(&q_messageToMon, ImageToBeSend);  // ImageToBeSend will be deleted by sendToMon

        } else if (not iCanAcquire){
            rt_sem_p(&sem_capture, TM_INFINITE);
        } else {
            cout << "Images acquisition is not possible !" << endl << flush ;
        }
    }
}

/**
 * Handle the arena search (needeed to calculate the position)
 */
void Tasks::getArena(void *arg) {
    
    int isRobotStarted ;
    bool isCameraOpen ;
    MessageImg *ArenaToBeSend ;
    Arena arenaTMP ;
    Img *image ;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task ArenaAndPosition starts here                                                    */
    /**************************************************************************************/
    cout << "   Start of the task" << __PRETTY_FUNCTION__ << endl << flush;
    
    //rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while(1) {
        rt_sem_p(&sem_arena, TM_INFINITE); // Allow to wait on the user demand for a search of an arena
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        isRobotStarted = robotStarted ;
        rt_mutex_release(&mutex_robotStarted) ;
        
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        isCameraOpen = cam->IsOpen();
        rt_mutex_release(&mutex_camera);

        if (isRobotStarted ==1 && isCameraOpen) {
            
            // Freezing capture
            rt_mutex_acquire(&mutex_capture,TM_INFINITE);
            sendCapture = false ;
            rt_mutex_release(&mutex_capture);
            
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            image = new Img(cam->Grab()) ; // It gets an image in order to search for an arena
            rt_mutex_release(&mutex_camera);

            if (arenaTMP.IsEmpty()) { // True if no arena has been found, false otherwise
                cout << "Searching for an arena..." << endl << flush ;
                arenaTMP = image->SearchArena() ; // It returns an arena object with coordinate of outline, empty if no arena found
            } else {
                cout << "An arena has been found !" << endl << flush ;

                // Show the user the arena found on the monitor
                image->DrawArena(arenaTMP) ;
                ArenaToBeSend = new MessageImg(MESSAGE_CAM_IMAGE, image) ;
                while (not isArenaConfirmed) {
                    rt_mutex_acquire(&mutex_monitor,TM_INFINITE);
                    monitor.Write(ArenaToBeSend) ;
                    rt_mutex_release(&mutex_monitor);
                    WriteInQueue(&q_messageToMon, ArenaToBeSend);  // ImageToBeSend will be deleted by sendToMon
                }

                // Waiting that the user validates or not the arena
                rt_sem_p(&sem_confirmArena, TM_INFINITE) ;
                if (isArenaConfirmed) {
                    cout << "Arena Validated !" << endl << flush ;
                    arena = arenaTMP ;
                } else {
                    cout << "Arena NOT Validated !" << endl << flush ;
                }
            }
            
            // Unfreezing capture
            rt_mutex_acquire(&mutex_capture,TM_INFINITE);
            sendCapture = true ;
            rt_mutex_release(&mutex_capture);
            rt_sem_v(&sem_capture) ;
            
            //////////////////////
            //  AUTRE TACHE ??  //
            //////////////////////
            
            // Calculation of the position
//            list<Position> positionRobot = image->SearchRobot(arena);
//            
//            if (not positionRobot.empty()) {
//                image->DrawAllRobots(positionRobot) ;
//            } else {
//                Position noPositionFound = Position() ;
//                noPositionFound.center.x = -1 ;
//                noPositionFound.center.y = -1 ;
//                
//                image->DrawRobot(noPositionFound) ;
//            }
//            
//            MessageImg *ImageToBeSendBis = new MessageImg(MESSAGE_CAM_IMAGE, image) ;
//            rt_mutex_acquire(&mutex_monitor,TM_INFINITE);
//            monitor.Write(ImageToBeSendBis) ;
//            rt_mutex_release(&mutex_monitor);
//            WriteInQueue(&q_messageToMon, ImageToBeSendBis);  // ImageToBeSend will be deleted by sendToMon
        }
    }
}
