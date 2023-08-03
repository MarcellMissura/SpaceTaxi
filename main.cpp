#include "SpaceTaxi.h"
#include "Experimenter.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    if (argc > 1) // Command line mode.
    {
        QCoreApplication a(argc, argv);
        Experimenter e;
        e.init();
        e.start();
        a.connect(&e, SIGNAL(finished()), &a, SLOT(quit()));
        return a.exec();
    }
    else // GUI mode
    {
        QApplication a(argc, argv);
        SpaceTaxi w;

        // Install the main window as an event filter.
        a.installEventFilter(&w);

        // Apply a stylesheet to the application.
        QFile file("styles.css");
        file.open(QFile::ReadOnly);
        QString styleSheet = QLatin1String(file.readAll());
        w.setStyleSheet(styleSheet);
        w.show();
        return a.exec();
    }
}
