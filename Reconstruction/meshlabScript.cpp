#include "meshlabScript.h"
#include <iostream>
#include <QDebug>
meshlabScript::meshlabScript()
{
	mergeCloseVetices = false;
	mergeThr = 4.0551e-05;

	removeIsoFace = false;
	isoThr = 0.1;

	quadEdgeCollapse = false;
	targetFaceNum = 3000;
}
meshlabScript::~meshlabScript()
{

}

void meshlabScript::generateScript(QString name)
{

	if (name.isEmpty())
		return;

	try
	{
		QFile file(name);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			qDebug() << "Can't open the file!" << endl;
		}
		QTextStream out(&file);

		out << "<!DOCTYPE FilterScript>" << endl
			<< "<FilterScript>" << endl;

		if (mergeCloseVetices)
			applyMergeCloseVetices(out);
		if (removeIsoFace)
			applyRemoveIsoFace(out);
		if (quadEdgeCollapse)
			applyQuadEdgeCollapse(out);
		if (removeIsoFace)
			applyRemoveIsoFace(out);
		out << "</FilterScript>" << endl;
	}
	catch (std::string s)
	{
		std::cout << s << std::endl;
	}
	
}

void meshlabScript::addParam(QTextStream &out, QString type, QString value, QString name)
{
	out << "  <Param"
		<< " type = \"" << type << "\""
		<< " value = \"" << value << "\""
		<< " name = \"" << name << "\""
		<< "  />" << endl;
}

void meshlabScript::addParamWitnMinMax(QTextStream & out, QString type, QString value, QString name, QString min, QString max)
{
	out << "  <Param"
		<< " type = \"" << type << "\""
		<< " value = \"" << value << "\""
		<< " name = \"" << name << "\""
		<< " min = \"" << min << "\""
		<< " max = \"" << max << "\""
		<< "  />" << endl;
}

void meshlabScript::applyMergeCloseVetices(QTextStream & out)
{
	QString filterName, ParamName, value, type, min, max;
	filterName = "Merge Close Vertices";
	
	out << " <filter name=\"" << filterName << "\""
		<< ">" << endl;

	ParamName = "Threshold";
	type = "RichAbsPerc";
	value = QString::number(mergeThr, 'g');
	min = "0";
	max = "0.00552249";
	addParamWitnMinMax(out, type, value, ParamName, min, max);

	out << " </filter>" << endl;
}

void meshlabScript::applyRemoveIsoFace(QTextStream & out)
{
	QString filterName, ParamName, value,type, min, max;
	filterName = "Remove Isolated pieces (wrt Diameter)";
	out << " <filter name=\"" << filterName << "\""
		<< ">" << endl;

	
	ParamName = "MinComponentDiag";
	value = QString::number(isoThr);
	type = "RichFloat";
	min = "0";
	max = "0.552249";
	addParamWitnMinMax(out, type, value, ParamName, min, max);

	out	<< " </filter>" << endl;
}

void meshlabScript::applyQuadEdgeCollapse(QTextStream & out)
{
	QString filterName, ParamName, value, type;
	filterName = "Quadric Edge Collapse Decimation";
	
	out << " <filter name=\"" << filterName << "\""
		<< ">" << endl;

	ParamName = "TargetFaceNum";
	value = QString::number(targetFaceNum);
	type = "RichInt";
	addParam(out, type, value, ParamName);

	ParamName = "TargetPerc";
	value = QString::number(0);
	type = "RichFloat";
	addParam(out, type, value, ParamName);

	ParamName = "QualityThr";
	value = QString::number(0.3);
	type = "RichFloat";
	addParam(out, type, value, ParamName);

	ParamName = "PreserveBoundary";
	value = "false";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "BoundaryWeight";
	value = QString::number(1);
	type = "RichFloat";
	addParam(out, type, value, ParamName);

	ParamName = "PreserveNormal";
	value = "true";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "PreserveTopology";
	value = "false";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "OptimalPlacement";
	value = "true";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "PlanarQuadric";
	value = "false";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "QualityWeight";
	value = "false";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "AutoClean";
	value = "true";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	ParamName = "Selected";
	value = "false";
	type = "RichBool";
	addParam(out, type, value, ParamName);

	out << " </filter>" << endl;
}
