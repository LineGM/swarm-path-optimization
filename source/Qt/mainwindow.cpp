#include "Qt/mainwindow.h"
#include "ui_mainWindow.h"
#include "PSO.hpp"
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QPixmap>
#include <filesystem>
#include <cstdlib>

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

void setDisabledOptions(Ui::mainWindow *mw)
{
	mw->startOptimization->setEnabled(false);
	mw->showResults->setEnabled(false);
	mw->saveResults->setEnabled(false);
}

mainWindow::mainWindow(QMainWindow *parent) : QMainWindow(parent)
{
	this->mw = new Ui::mainWindow;
	mw->setupUi(this);
	setIcons(mw);
	setDisabledOptions(mw);
}

void mainWindow::on_Exit_triggered()
{
	QApplication::quit();
}

void mainWindow::on_obstaclesMapOpen_triggered()
{
	obstacles_center_x.clear();
	obstacles_center_y.clear();

	QString fileName = QFileDialog::getOpenFileName(this, tr("Открыть карту препятствий"), "", tr("Map (*.map)"));
	if (!fileName.isEmpty())
	{
		PSOLibrary::Obstacles::readObstaclesMap(std::filesystem::path(fileName.toStdString()), obstacles_center_x, obstacles_center_y, OptimizationParams);
		QString obs = "Полученные из карты координаты препятствий:\n";
		for (auto i = 0; i < OptimizationParams.NUM_OBSTACLE; i++)
		{
			obs += "{" + QString::number(obstacles_center_x[static_cast<size_t>(i)]) + ", " + QString::number(obstacles_center_y[static_cast<size_t>(i)]) + "}\n";
		}
		QMessageBox::information(this, "Считанные препятствия", obs);
		mw->startOptimization->setEnabled(true);
	}
	else
	{
		QMessageBox::warning(this, "Ошибка открытия карты препятствий", "Не удалось получить путь для открытия карты препятствий.");
	}
}

void mainWindow::on_obstaclesMapGenerate_triggered()
{
	obstacles_center_x.clear();
	obstacles_center_y.clear();

	int seed = 123;
	PSOLibrary::Obstacles::generate_obstacles(obstacles_center_x, obstacles_center_y, seed, OptimizationParams);
	
	QString obs = "Сгенерированные координаты препятствий:\n";
	for (auto i = 0; i < OptimizationParams.NUM_OBSTACLE; i++)
	{
		obs += "{" + QString::number(obstacles_center_x[static_cast<size_t>(i)]) + ", " + QString::number(obstacles_center_y[static_cast<size_t>(i)]) + "}\n";
	}
	
	QMessageBox::information(this, "Сгенерированные препятствия", obs);
	mw->startOptimization->setEnabled(true);
}

void mainWindow::on_setParams_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Открыть файл параметров"), "", tr("Params File (*.prm)"));
	if (!fileName.isEmpty())
	{
		PSOLibrary::Algorithm::readParams(std::filesystem::path(fileName.toStdString()), OptimizationParams);
		QString obs = "Полученные из файла параметры оптимизации:\n";
		obs += "SWARM_SIZE = " + QString::number(OptimizationParams.SWARM_SIZE) + "\n";
		obs += "NO_OF_ITERS = " + QString::number(OptimizationParams.NO_OF_ITERS) + "\n";
		obs += "WMIN = " + QString::number(OptimizationParams.WMIN) + "\n";
		obs += "WMAX = " + QString::number(OptimizationParams.WMAX) + "\n";
		obs += "C1 = " + QString::number(OptimizationParams.C1) + "\n";
		obs += "C2 = " + QString::number(OptimizationParams.C2) + "\n";
		obs += "START_X = " + QString::number(OptimizationParams.START_X) + "\n";
		obs += "START_Y = " + QString::number(OptimizationParams.START_Y) + "\n";
		obs += "DESTINATION_X = " + QString::number(OptimizationParams.DESTINATION_X) + "\n";
		obs += "DESTINATION_Y = " + QString::number(OptimizationParams.DESTINATION_Y) + "\n";
		obs += "LOWER_BOUNDARY = " + QString::number(OptimizationParams.LOWER_BOUNDARY) + "\n";
		obs += "UPPER_BOUNDARY = " + QString::number(OptimizationParams.UPPER_BOUNDARY) + "\n";
		obs += "NUM_OBSTACLE = " + QString::number(OptimizationParams.NUM_OBSTACLE) + "\n";
		obs += "R = " + QString::number(OptimizationParams.R) + "\n";
		obs += "TARGET_TOLERANCE = " + QString::number(OptimizationParams.TARGET_TOLERANCE) + "\n";
		obs += "V_MAX = " + QString::number(OptimizationParams.V_MAX) + "\n";
		obs += "POS_MULTIPLE = " + QString::number(OptimizationParams.POS_MULTIPLE) + "\n";
		obs += "VEL_MULTIPLE = " + QString::number(OptimizationParams.VEL_MULTIPLE) + "\n";
		obs += "LOCAL_CONV_TOLERANCE = " + QString::number(OptimizationParams.LOCAL_CONV_TOLERANCE);

		QMessageBox::information(this, "Считанные препятствия", obs);
	}
	else
	{
		QMessageBox::warning(this, "Ошибка открытия файла параметров", "Не удалось получить путь для открытия файла параметров.");
	}
}

void mainWindow::on_startOptimization_triggered()
{
	Coord target = Coord();
	target.x = OptimizationParams.DESTINATION_X;
	target.y = OptimizationParams.DESTINATION_Y;

	Coord start = Coord();
	start.x = OptimizationParams.START_X;
	start.y = OptimizationParams.START_Y;

	PSOLibrary::Algorithm::particleSwarmOptimization(start, target, obstacles_center_x, obstacles_center_y, OptimizationParams);
	QMessageBox::information(this, "Оптимизация", "Оптимизация завершена!\nДля отображения результатов оптимизации выберете соответствущий пункт в меню или используйте сочетание клавиш Ctrl+H.");
	mw->showResults->setEnabled(true);
	mw->saveResults->setEnabled(true);
}

void mainWindow::on_showResults_triggered()
{
	std::system("python3 ./plot_csv.py");
	QPixmap resultsImage;
	resultsImage.load("./optimization_results.png");
	mw->showResultsLabel->setPixmap(resultsImage);
	mw->showResultsLabel->setScaledContents(true);
	mw->saveResults->setEnabled(true);
}

void mainWindow::on_saveResults_triggered()
{
	QPixmap pm = mw->showResultsLabel->pixmap(Qt::ReturnByValue);

	if (!pm.isNull())
	{
		QString fileNameToSave = QFileDialog::getSaveFileName(this, tr("Сохранение результатов оптимизации"), "", tr("Images (*.png)"));
		if (!fileNameToSave.isEmpty())
		{
			pm.save(fileNameToSave);
		}
		else
		{
			QMessageBox::warning(this, "Ошибка сохранения результатов", "Не удалось получить путь для сохранения результатов.");
		}
	}
	else
	{
		QMessageBox::warning(this, "Ошибка сохранения результатов", "Не удалось получить результаты для сохранения.\nПопробуйте начать оптимизацию, чтобы получить результаты.");
	}
}

void mainWindow::on_aboutProgram_triggered()
{
	QMessageBox::information(this, "О программе", "Автор программы: Гудков Глеб Олегович\nГод создания: 2024\nДанная программа создана в рамках выпускной квалификационной работы на степень бакалавра по специальности и профилю:\n\"Информатика и вычислительная техника\"\n\"Искусственный интеллект и системы автоматизированного проектирования\"");
}

void mainWindow::on_aboutQt_triggered()
{
	QApplication::aboutQt();
}

mainWindow::~mainWindow()
{
	delete mw;
}
