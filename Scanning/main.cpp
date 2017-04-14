#include "scanning.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Scanning w;
	w.show();
	return a.exec();
}
