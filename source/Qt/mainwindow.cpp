#include "Qt/mainwindow.h"
#include "ui_mainWindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <filesystem>

void setIcons(Ui::mainWindow *mw)
{
	// Set icons to menu options
	mw->obstaclesMapOpen->setIcon(QIcon("../../source/Qt/icons/obstaclesMapOpen.png"));
	mw->obstaclesMapGenerate->setIcon(QIcon("../../source/Qt/icons/obstaclesMapGenerate.png"));
	mw->Exit->setIcon(QIcon("../../source/Qt/icons/exit.png"));
	mw->setParams->setIcon(QIcon("../../source/Qt/icons/set_params.png"));
	mw->startOptimization->setIcon(QIcon("../../source/Qt/icons/start_optimization.png"));
	mw->showResults->setIcon(QIcon("../../source/Qt/icons/show_results.png"));
	mw->saveResults->setIcon(QIcon("../../source/Qt/icons/save_results.png"));
	mw->aboutProgram->setIcon(QIcon("../../source/Qt/icons/about_program.png"));
	mw->aboutQt->setIcon(QIcon("../../source/Qt/icons/about_qt.png"));
}

mainWindow::mainWindow(QMainWindow *parent) : QMainWindow(parent)
{
	this->mw = new Ui::mainWindow;
	mw->setupUi(this);
	setIcons(mw);
}

mainWindow::~mainWindow()
{
	delete mw;
}