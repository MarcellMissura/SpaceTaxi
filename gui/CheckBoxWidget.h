#ifndef CHECKBOXWIDGET_H_
#define CHECKBOXWIDGET_H_
#include <QtGui>
#include <QTreeWidget>

class CheckBoxWidget: public QTreeWidget
{
	Q_OBJECT

public:
	CheckBoxWidget(QWidget *parent = 0);
	~CheckBoxWidget();

	void init();

public slots:
	void checkBoxChanged(QTreeWidgetItem*, int);
	void scrollOnExpand(QTreeWidgetItem*);

signals:
	void stateMemberStatusChanged(int, bool);

private:
	void saveState(QTreeWidgetItem*);
};

#endif // CHECKBOXWIDGET_H_
