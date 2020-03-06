#ifndef LIFTOFF_CLI_DATA_PLOTTER_H
#define LIFTOFF_CLI_DATA_PLOTTER_H

#include <RQ_OBJECT.h>
#include <TCanvas.h>
#include <TApplication.h>
#include <TGraph.h>

static const char *ROOT_CLS_NAME = "data_plotter";

class data_plotter {
RQ_OBJECT(ROOT_CLS_NAME)
private:
    bool valid{true};

    TApplication &app;
    TCanvas *canvas;

    int pad_idx_counter{1};
    std::vector<TGraph *> graphs;
public:
    data_plotter(TApplication &ta, const std::string &app_name,
                 int nx = 1, int ny = 1);

    ~data_plotter();

    bool is_valid();

    void set_valid(bool validity);

    TCanvas *get_canvas();

    int add_plot(TGraph *g);

    TGraph *get_plot(int gid);

    void update_plots();

    void update_plot(int gid);

    void await(long usec);

    void signal_close();
};

void plotter_handle_gui(bool yield);

#endif // LIFTOFF_CLI_DATA_PLOTTER_H
