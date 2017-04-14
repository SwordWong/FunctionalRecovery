/********************************************************************************
** Form generated from reading UI file 'scanning.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SCANNING_H
#define UI_SCANNING_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ScanningClass
{
public:
    QWidget *centralWidget;
    QLabel *label_color;
    QLabel *label_normal;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *ScanningClass)
    {
        if (ScanningClass->objectName().isEmpty())
            ScanningClass->setObjectName(QStringLiteral("ScanningClass"));
        ScanningClass->resize(1377, 730);
        centralWidget = new QWidget(ScanningClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label_color = new QLabel(centralWidget);
        label_color->setObjectName(QStringLiteral("label_color"));
        label_color->setGeometry(QRect(0, 0, 640, 480));
        label_color->setAutoFillBackground(true);
        label_normal = new QLabel(centralWidget);
        label_normal->setObjectName(QStringLiteral("label_normal"));
        label_normal->setGeometry(QRect(640, 0, 640, 480));
        label_normal->setAutoFillBackground(true);
        ScanningClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(ScanningClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1377, 23));
        ScanningClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(ScanningClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        ScanningClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(ScanningClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        ScanningClass->setStatusBar(statusBar);

        retranslateUi(ScanningClass);

        QMetaObject::connectSlotsByName(ScanningClass);
    } // setupUi

    void retranslateUi(QMainWindow *ScanningClass)
    {
        ScanningClass->setWindowTitle(QApplication::translate("ScanningClass", "Scanning", 0));
        label_color->setText(QString());
        label_normal->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ScanningClass: public Ui_ScanningClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCANNING_H
