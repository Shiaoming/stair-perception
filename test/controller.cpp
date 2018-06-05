#include "controller.h"

Controller::Controller()
{
    reader = new Reader;

    connect(this, &Controller::prev_signal, reader, &Reader::read_file_prev);
    connect(this, &Controller::next_signal, reader, &Reader::read_file_next);
    connect(this, &Controller::process_pause, reader, &Reader::update_pause);

    processor = new DProcessor();

    connect(this, &Controller::process_pause, processor, &DProcessor::update_pause);

    reader->start();
    processor->start();
}

Controller::~Controller()
{
    if (reader->isRunning())
    {
        stop_reader_thread();
        quit_reader_thread();
    }

    if (processor->isRunning())
    {
        stop_process_thread();
        quit_process_thread();
    }
}


void Controller::run_mode_update(RunModeDef mode)
{
    run_mode = mode;
}

void Controller::start_bt_signal()
{
    if (run_mode == files_mode)
            emit prev_signal();

    if (!processor->misRunning())
        processor->start();

    emit process_pause(false);
}

void Controller::stop_bt_signal()
{
    if (run_mode == files_mode)
            emit next_signal();
    else
    {
        emit process_pause(true);
    }
}


void Controller::stop_reader_thread()
{
    reader->stop();
}

void Controller::quit_reader_thread()
{
    reader->quit();
    reader->wait();
}


void Controller::stop_process_thread()
{
    processor->stop();
}

void Controller::quit_process_thread()
{
    processor->quit();
    processor->wait();
}


