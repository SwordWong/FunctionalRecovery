#ifndef FUNCTIONALRECOVERY_H
#define FUNCTIONALRECOVERY_H

#include <QtWidgets/QMainWindow>
#include "ui_functionalrecovery.h"

class FunctionalRecovery : public QMainWindow
{
	Q_OBJECT

public:
	FunctionalRecovery(QWidget *parent = 0);
	~FunctionalRecovery();

private:
	Ui::FunctionalRecoveryClass ui;
};

#endif // FUNCTIONALRECOVERY_H
