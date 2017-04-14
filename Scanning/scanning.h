#ifndef SCANNING_H
#define SCANNING_H

#include <QtWidgets/QMainWindow>
#include "ui_scanning.h"

class Scanning : public QMainWindow
{
	Q_OBJECT

public:
	Scanning(QWidget *parent = 0);
	~Scanning();

	void timerEvent(QTimerEvent* ev);
private:
	Ui::ScanningClass ui;

	QString m_currentPath;
	int m_fpsTimerId;
	int m_frameIndex;
};

#endif // SCANNING_H
