/********************************************************************************
** Form generated from reading UI file 'EITPlugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EITPLUGIN_H
#define UI_EITPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_EITPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *vboxLayout;
    QLabel *ui_sample_label;
    QPushButton *ui_home_button;

    void setupUi(QDockWidget *EITPlugin)
    {
        if (EITPlugin->objectName().isEmpty())
            EITPlugin->setObjectName(QStringLiteral("EITPlugin"));
        EITPlugin->resize(400, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        vboxLayout = new QVBoxLayout(dockWidgetContents);
        vboxLayout->setObjectName(QStringLiteral("vboxLayout"));
        vboxLayout->setAlignment(Qt::AlignTop);
        ui_sample_label = new QLabel(dockWidgetContents);
        ui_sample_label->setObjectName(QStringLiteral("ui_sample_label"));
        ui_sample_label->setTextFormat(Qt::RichText);

        vboxLayout->addWidget(ui_sample_label);

        ui_home_button = new QPushButton(dockWidgetContents);
        ui_home_button->setObjectName(QStringLiteral("ui_home_button"));

        vboxLayout->addWidget(ui_home_button);

        EITPlugin->setWidget(dockWidgetContents);

        retranslateUi(EITPlugin);

        QMetaObject::connectSlotsByName(EITPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *EITPlugin)
    {
        EITPlugin->setWindowTitle(QApplication::translate("EITPlugin", "EiT Plugin", Q_NULLPTR));
        ui_sample_label->setText(QApplication::translate("EITPlugin", "Sample label", Q_NULLPTR));
        ui_home_button->setText(QApplication::translate("EITPlugin", "Home", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class EITPlugin: public Ui_EITPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EITPLUGIN_H
