#include "widget.h"

#include <QApplication>
#include <vtkOutputWindow.h>
#include <QWidget.h>
#include "QVTKOpenGLWidget.h"
#include "QVTKOpenGLNativeWidget.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    Widget w;
    w.setWindowTitle(QString("Coal Stack Intelligent Inspection System"));
    w.show();
    return a.exec();
}
