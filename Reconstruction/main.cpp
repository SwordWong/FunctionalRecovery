#include <QtWidgets/QApplication>
#include <glut.h>
#include "reconstruction.h"
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Reconstruction w;
	w.show();
	//glutInit(&argc, argv);
	return a.exec();
}
