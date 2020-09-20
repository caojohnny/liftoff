#ifndef LIFTOFF_CLI_DATA_PLOTTER_H
#define LIFTOFF_CLI_DATA_PLOTTER_H

#include <RQ_OBJECT.h>
#include <TCanvas.h>
#include <TApplication.h>
#include <TGraph.h>

static const char *ROOT_CLS_NAME = "data_plotter";

/**
 * Represents a ROOT plotter window.
 */
class data_plotter {
RQ_OBJECT(ROOT_CLS_NAME)
private:
    bool valid{true};

    TApplication &app;
    TCanvas *canvas;

    int pad_idx_counter{1};
    std::vector<TGraph *> graphs;
public:
    /**
     * Creates a new plotter window with the given
     * application, window name and plotter panes.
     *
     * @param ta the ROOT application
     * @param app_name the name of the window
     * @param nx the number of horizontal panes
     * @param ny the number of vertical panes
     */
    data_plotter(TApplication &ta, const std::string &app_name,
                 int nx = 1, int ny = 1);

    /**
     * The destructor used to clean-up resources.
     */
    ~data_plotter();

    /**
     * Determines whether the window is still valid.
     *
     * @return true if the window is valid
     */
    bool is_valid();

    /**
     * Changes the validity of the window.
     *
     * @param validity whether or not the window is valid
     */
    void set_valid(bool validity);

    /**
     * Obtains the window canvas.
     *
     * @return the window ROOT canvas object
     */
    TCanvas *get_canvas();

    /**
     * Adds a plot to the ROOT window.
     *
     * @param g the plot to add
     * @return the GID of the plot in this window
     */
    int add_plot(TGraph *g);

    /**
     * Performs a lookup of the plot based upon the GID
     * value returned from adding it to this window.
     *
     * @param gid the GID value
     * @return the plot, if valid
     */
    TGraph *get_plot(int gid);

    /**
     * Updates all plots added to this window.
     */
    void update_plots();

    /**
     * Updates just the plot with the given GID.
     *
     * @param gid the GID returned from adding the plot to
     * this window
     */
    void update_plot(int gid);

    /**
     * Pauses the program but still performs GUI processing
     * in the background to prevent it from freezing during
     * a sleep.
     *
     * @param usec the number of microseconds to sleep
     */
    void await(long usec);

    /**
     * Signal handler for the ROOT window close.
     */
    void signal_close();
};

/**
 * Handles the currently queued GUI events for a tight loop.
 *
 * @param yield whether or not extra time should be taken
 * to yield to reduce CPU usage
 */
void plotter_handle_gui(bool yield);

#endif // LIFTOFF_CLI_DATA_PLOTTER_H
