#include "FileLoader.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

FileLoader::FileLoader(QString fileName)
{
	load(fileName);
}

FileLoader::~FileLoader()
{

}

int FileLoader::lines()
{
	return data.size();
}

void FileLoader::load(QString fileName)
{
	QString line;
	QStringList tokenList;
	bool ok;
	double value;
	int idx = 0;

	// Open the data file.
	QFile file;
	file.setFileName(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Could not open" << fileName;
		return;
	}
	QTextStream in(&file);

	data.clear();

	int lineLength = 0;
	while (!in.atEnd())
	{
		line = in.readLine();
		if (line.startsWith("//") or line.startsWith('#'))
			continue;

		tokenList = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
		if (lineLength > 0 and tokenList.size() < lineLength)
			continue;
		lineLength = tokenList.size();

		data << QList<double>();
		for (int i = 0; i < tokenList.size(); i++)
		{
			value = tokenList[i].toDouble(&ok);
			if (ok)
				data.last() << value;
			else
				qDebug() << "Could not read token" << i << "\"" << tokenList[i] << "\" in line" << idx;
		}

		idx++;
	}

	file.close();
}

QList<double>& FileLoader::operator[](int i)
{
	return data[i];
}


