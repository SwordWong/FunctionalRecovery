#include "functionalrecovery.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	FunctionalRecovery w;
	w.show();
	return a.exec();
}
