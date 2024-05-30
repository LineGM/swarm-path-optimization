#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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


private:
    Ui::mainWindow *mw;
};

typedef std::shared_ptr<mainWindow>MainWindowSharedPtr;

#endif // MAINWINDOW_H
