#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"
#include "reader.h"
#include "processor.h"

#include <QThread>
#include <QtCore>

class Controller : public QObject
{
    Q_OBJECT
    
public:
//     QThread *reader_thread,*process_thread;
    Reader *reader;
    DProcessor *processor;
    
public:
    Controller() ;
    ~Controller() override;
    
private:
    void stop_reader_thread();
    void quit_reader_thread();

    void stop_process_thread();
    void quit_process_thread();
    
public slots:
    void run_mode_update(RunModeDef);
    void start_bt_signal();
    void stop_bt_signal();    
    
signals:
    void prev_signal();
    void next_signal();
    
    void process_pause(bool);

private:
    RunModeDef run_mode;
};

#endif