#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Params.hpp"

#if (defined(__WIN32) || defined(__MINGW32__) || defined(__MINGW64__) || defined(__CYGWIN__))
// #include <QtPlugin>
// Q_IMPORT_PLUGIN(QWindowsIntegrationPlugin)
#endif

namespace Ui {
class mainWindow;
}

class mainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainWindow(QMainWindow *parent = 0);
    ~mainWindow();

private slots:
    void on_obstaclesMapOpen_triggered();
    void on_obstaclesMapGenerate_triggered();
    void on_Exit_triggered();
    void on_setParams_triggered();
    void on_startOptimization_triggered();
    void on_showResults_triggered();
    void on_saveResults_triggered();
    void on_aboutProgram_triggered();
    void on_aboutQt_triggered();

private:
    Ui::mainWindow *mw;
    Params OptimizationParams;
    std::vector<double> obstacles_center_x;
    std::vector<double> obstacles_center_y;
};

typedef std::shared_ptr<mainWindow>MainWindowSharedPtr;

#endif // MAINWINDOW_H
