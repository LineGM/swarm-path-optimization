#include <QApplication>
#include <Qt/mainwindow.h>

auto main(int argc, char *argv[]) -> int
{
	QApplication App(argc, argv);
	mainWindow Window;

	Window.setWindowIcon(QIcon("../../source/Qt/icons/main_window_icon.png"));

	Window.show();

	return App.exec();
}