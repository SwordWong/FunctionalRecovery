#include "functionalrecovery.h"
#include "FunctionRecoveryDataHolder.h"
#include <QFileDialog>
FunctionalRecovery::FunctionalRecovery(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	connectEvent();
}

FunctionalRecovery::~FunctionalRecovery()
{

}

void FunctionalRecovery::clicked_pb_oversegement()
{
	fr_data_holder.oversegement_mesh();
}


void FunctionalRecovery::clicked_pb_load_mesh()
{
	QString file = QFileDialog::getOpenFileName(0, tr("load mesh"),
		tr("."), tr("*.obj"));
	if (file.isEmpty())
		return;
	fr_data_holder.m_mesh.loadObj(file.toStdString().c_str(), true, false);
	ui.mesh_widget->setMesh(&fr_data_holder.m_mesh);
}

void FunctionalRecovery::connectEvent()
{
	connect(ui.pb_load_mesh, SIGNAL(clicked()), this, SLOT(clicked_pb_load_mesh()));
	connect(ui.pb_oversegement, SIGNAL(clicked()), this, SLOT(clicked_pb_oversegement()));
}
