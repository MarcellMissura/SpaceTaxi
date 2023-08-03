#ifndef FILELOADER_H_
#define FILELOADER_H_
#include <QList>

class FileLoader
{
	QList< QList<double> > data;

public:
	FileLoader(QString fileName);
	~FileLoader();

	int lines();
	void load(QString fileName);

	QList<double>& operator[](int i);
};

#endif /* FILELOADER_H_ */
