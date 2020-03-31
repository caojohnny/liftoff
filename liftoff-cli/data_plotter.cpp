#include "data_plotter.h"

#include <TCanvas.h>
#include <TApplication.h>
#include <TRootCanvas.h>
#include <TSystem.h>
#include <chrono>
#include <thread>

static const int YIELD_INTERVAL_USEC = 20000;

data_plotter::data_plotter(TApplication &app, const std::string &app_name,
                           int nx, int ny) : app(app) {
    const char *app_name_carr = app_name.c_str();
    auto *tc = new TCanvas(app_name_carr, app_name_carr);
    auto *rc = (TRootCanvas *) tc->GetCanvasImp();
    rc->Connect("CloseWindow()", ROOT_CLS_NAME, this, "signal_close()");

    canvas = tc;
    canvas->Divide(nx, ny);
}

data_plotter::~data_plotter() {
    for (auto &graph : graphs) {
        delete graph;
    }
}

bool data_plotter::is_valid() {
    return valid;
}

void data_plotter::set_valid(bool validity) {
    valid = validity;
}

TCanvas *data_plotter::get_canvas() {
    return canvas;
}

int data_plotter::add_plot(TGraph *g) {
    graphs.push_back(g);

    int gid = pad_idx_counter;
    pad_idx_counter++;

    canvas->cd(gid);
    g->Draw();
    return gid;
}

TGraph *data_plotter::get_plot(int gid) {
    return graphs.at(gid - 1);
}

void data_plotter::update_plots() {
    for (int i = 1; i < pad_idx_counter; ++i) {
        update_plot(i);
    }
}

void data_plotter::update_plot(int gid) {
    canvas->cd(gid);
    canvas->Update();
    canvas->Modified();
    canvas->Draw();
}

void data_plotter::await(long usec) {
    auto begin_time = std::chrono::steady_clock::now();
    while (is_valid()) {
        auto cur_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - begin_time);
        if (duration.count() >= usec) {
            break;
        }

        plotter_handle_gui(true);
    }
}

void data_plotter::signal_close() {
    valid = false;
}

void plotter_handle_gui(bool yield) {
    gSystem->ProcessEvents();

    if (yield) {
        usleep(YIELD_INTERVAL_USEC);
    }
}
