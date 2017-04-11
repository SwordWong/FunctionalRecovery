#pragma once
#include <QString>
#include <QFile>
#include <QTextStream>
class meshlabScript
{
public:
	meshlabScript();
	~meshlabScript();

	void generateScript(QString name);
protected:
	void addParam(QTextStream &out, QString type, QString value, QString name);
	void addParamWitnMinMax(QTextStream &out, QString type, QString value, QString name, QString min, QString max);

	void applyMergeCloseVetices(QTextStream &out);
	void applyRemoveIsoFace(QTextStream &out);
	void applyQuadEdgeCollapse(QTextStream &out);

public:
	bool mergeCloseVetices;
	float mergeThr;

	bool removeIsoFace;
	float isoThr;

	bool quadEdgeCollapse;
	int targetFaceNum;
};
