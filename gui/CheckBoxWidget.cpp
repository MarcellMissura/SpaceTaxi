#include <QSettings>
#include <QHeaderView>
#include "CheckBoxWidget.h"
#include "blackboard/State.h"

// The checkbox widget contains a checkbox for every registered member of the state object.
// The registered state variables are structured into a tree based on the identifier string
// that was used for registration. A "." in the identifier creates a parent-child relation.
// When the checkbox is pressed, it signals the id and the on/off state of the member.
// This functionality is used to determine if the time series of a state member is shown on
// the graph widget or not.

CheckBoxWidget::CheckBoxWidget(QWidget *parent)
    : QTreeWidget(parent)
{
//	setStyle(new QCleanlooksStyle());
	setColumnCount(1);
    header()->hide();
	setAnimated(true);
	setSelectionMode(QTreeWidget::NoSelection);
	setFocusPolicy(Qt::NoFocus);

	connect(this, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkBoxChanged(QTreeWidgetItem*, int)));
	connect(this, SIGNAL(itemExpanded(QTreeWidgetItem*)), this, SLOT(scrollOnExpand(QTreeWidgetItem*)));
}

CheckBoxWidget::~CheckBoxWidget()
{
	// In the destructor the state of the checkboxes is saved for the next session.
	QSettings settings;
	settings.beginGroup("checkboxstates");
	settings.remove(""); //clear group
	settings.endGroup();

	saveState(invisibleRootItem());
}

// Prepares the tree structure for the checkbox widget.
// The checkbox widget is filled dynamically with a checkbox for every registered member of the State object.
// Checkbox states are restored from the last session.
// state.init() must be called first!
void CheckBoxWidget::init()
{
	QSettings settings;
	settings.beginGroup("checkboxstates");

	QTreeWidgetItem* root;
	bool parentFound = false;
	for (int i = 0; i < ::state.memberNames.length(); i++)
	{
		// Reset the root the root of the tree.
		root = invisibleRootItem();

		// Split the state member key up on the dot and try to register each part in the tree.
		QString stateMemberKey = ::state.memberNames[i];
		QStringList stateMemberKeyPartList = stateMemberKey.split(".");
		foreach (QString stateMemberKeyPart, stateMemberKeyPartList)
		{
			// First let's see if we already have a branch for this part of the key.
			parentFound = false;
			for (int j = 0; j < root->childCount(); j++)
			{
				QTreeWidgetItem* item = root->child(j);
				if (item->text(0) == stateMemberKeyPart)
				{
					// Matching tree node found. Recurse.
					root = item;
					parentFound = true;
					break;
				}
			}

			// No match found, create a new branch.
			if (!parentFound)
			{
				QTreeWidgetItem* newItem = new QTreeWidgetItem();
				newItem->setText(0, stateMemberKeyPart);
				if (stateMemberKeyPart == stateMemberKeyPartList.last())
				{
					newItem->setData(0, Qt::UserRole, i); // id
					newItem->setData(0, Qt::UserRole+1, stateMemberKey); // fq
					newItem->setCheckState(0, Qt::Unchecked);

					// If this member was saved as checked in the QSettings, restore checked state
					// and expand all parents up to the root.
					if (settings.contains(stateMemberKey) && settings.value(stateMemberKey).toBool())
					{
						newItem->setCheckState(0, Qt::Checked);

						QTreeWidgetItem* parent = root;
						while (parent != invisibleRootItem() && parent != 0)
						{
							parent->setExpanded(true);
							parent = parent->parent();
						}
					}
				}
				root->addChild(newItem);
				root = newItem;
			}
		}
	}

	settings.endGroup();
}

// Handles the clicks on the checkbox tree.
void CheckBoxWidget::checkBoxChanged(QTreeWidgetItem* it, int column)
{
	emit stateMemberStatusChanged(it->data(column, Qt::UserRole).toInt(), (it->checkState(column) == 2));
}

// When an item is expanded, this slot makes sure that the last child is still visible.
void CheckBoxWidget::scrollOnExpand(QTreeWidgetItem* it)
{
	scrollToItem(it->child(it->childCount()-1));
}

// Saves the checkbox states in the QSettings.
void CheckBoxWidget::saveState(QTreeWidgetItem* it)
{
	QSettings settings;
	settings.beginGroup("checkboxstates");

	for (int i = 0; i < it->childCount(); i++)
	{
		QTreeWidgetItem* child = it->child(i);
		if (child->childCount() > 0)
		{
			saveState(child);
		}
		else
		{
			settings.setValue(child->data(0, Qt::UserRole+1).toString(), (child->checkState(0) == 2));
		}
	}

	settings.endGroup();
}
