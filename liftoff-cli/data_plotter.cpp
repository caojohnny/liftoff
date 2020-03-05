#include "data_plotter.h"

#include <TCanvas.h>
#include <TApplication.h>
#include <TRootCanvas.h>
#include <TSystem.h>
#include <chrono>
#include <thread>

data_plotter::data_plotter(TApplication &app, const std::string &app_name,
                           int nx, int ny) : app(app) {
    TCanvas *tc = new TCanvas("tc", app_name.c_str());
    TRootCanvas *rc = (TRootCanvas *) tc->GetCanvasImp();
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

void data_plotter::ensure_open_loop(bool yield) {
    app.StartIdleing();
    gSystem->ProcessEvents();
    app.StopIdleing();

    if (yield) {
        std::this_thread::yield();
    }
}

void data_plotter::await(long usec) {
    auto begin_time = std::chrono::steady_clock::now();
    while (is_valid()) {
        auto cur_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - begin_time);
        if (duration.count() >= usec) {
            break;
        }

        ensure_open_loop(true);
    }
}

void data_plotter::signal_close() {
    valid = false;
}
