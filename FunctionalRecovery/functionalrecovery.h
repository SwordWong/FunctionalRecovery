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
public slots:
	void clicked_pb_load_mesh();
	void clicked_pb_oversegement();
protected:
	
private:
	void connectEvent();
private:
	Ui::FunctionalRecoveryClass ui;
};

#endif // FUNCTIONALRECOVERY_H
