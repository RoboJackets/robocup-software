#include <PlayConfigTab.hpp>
#include <PlayConfigTab.moc>

#include <gameplay/Play.hpp>
#include <boost/foreach.hpp>

#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMenu>

using namespace std;
using namespace boost;
using namespace Gameplay;

Q_DECLARE_METATYPE(shared_ptr<Play>)

// Gets the Play for a list item
shared_ptr<Play> play_for_item(QTreeWidgetItem *item)
{
	return item->data(0, Qt::UserRole).value<shared_ptr<Play> >();
}

PlayConfigTab::PlayConfigTab(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);
	
	ui.plays->setContextMenuPolicy(Qt::CustomContextMenu);
	
	_iconRun = QIcon(":/icons/running.png");
// 	_iconWait = QIcon(":/icons/waiting.png");
}

void PlayConfigTab::setup(boost::shared_ptr<Gameplay::GameplayModule> gp)
{
	_gameplay = gp;
	
	typedef QMap<QString, QTreeWidgetItem *> CatMap;
	CatMap categories;
	
	BOOST_FOREACH(PlayFactoryBase *factory, *PlayFactoryBase::factories)
	{
		QTreeWidgetItem *parent;
		if (!factory->category.isNull())
		{
			CatMap::iterator i = categories.find(factory->category);
			if (i == categories.end())
			{
				// New category
				parent = new QTreeWidgetItem();
				parent->setText(0, factory->category);
				ui.plays->addTopLevelItem(parent);
				categories[factory->category] = parent;
			} else {
				// Existing category
				parent = i.value();
			}
		} else {
			parent = ui.plays->invisibleRootItem();
		}
		
		//FIXME - Either propagate the shared_ptr throughout all plays or make a regular pointer safe
		shared_ptr<Play> play = shared_ptr<Play>(factory->create(_gameplay.get()));
		QTreeWidgetItem *item = new QTreeWidgetItem(parent);
		item->setData(0, Qt::UserRole, QVariant::fromValue(play));
		item->setText(0, play->name());
		item->setCheckState(0, Qt::Unchecked);
		item->setIcon(1, _iconWait);
		
		_playMap[play->name()] = item;
	}
	
	ui.plays->sortItems(0, Qt::AscendingOrder);
	ui.plays->expandAll();
	ui.plays->resizeColumnToContents(0);
	ui.plays->resizeColumnToContents(1);
}

void PlayConfigTab::on_load_clicked()
{
	// get a filename from the user
	QString filename = QFileDialog::getOpenFileName(this,
	     tr("Load Playbook"), "./", tr("Playbook Files (*.pbk)"));

	load(filename);
}

void PlayConfigTab::load(QString filename)
{
	// Set the name of the playbook
	ui.playbookName->setText(QFileInfo(filename).baseName());
	
	// clear the playbook
	selectNone();

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		printf("Failed to open %s\n", (const char *)filename.toAscii());
		return;
	}
	
	QTextStream ts(&file);
	
	bool foundGoalie = false;
	while (!ts.atEnd())
	{
		QString name = ts.readLine();
		if (name != "" && !name.startsWith('#'))
		{
			if (name == "goalie")
			{
				foundGoalie = true;
			} else {
				enable(name);
			}
		}
	}

	useGoalie(foundGoalie);
}

void PlayConfigTab::enable(QString name)
{
	PlayMap::const_iterator i = _playMap.find(name);
	if (i == _playMap.end())
	{
		printf("Missing play \"%s\"\n", (const char *)name.toAscii());
	} else {
		// This will cause itemChanged to be emitted, which will actually enable the play.
		i.value()->setCheckState(0, Qt::Checked);
	}
}

void PlayConfigTab::useGoalie(bool value)
{
	// ensure that the button for the goalie is checked appropriately
	ui.goalie->setChecked(value);
}

void PlayConfigTab::on_save_clicked()
{
	// get a filename from the user
	QString filename = QFileDialog::getSaveFileName(this,
		     tr("Save Playbook"), "./", tr("Playbook Files (*.pbk)"));

	// change the name of the current playbook
	ui.playbookName->setText(QFileInfo(filename).baseName());

	// open the file
	QFile file(filename);
	QTextStream ts(&file);

	// if there is a goalie, write to file
	if (_gameplay->goalie())
	{
		ts << "goalie\n";
	}

	// Write the names of all enabled plays
	BOOST_FOREACH(shared_ptr<Play> play, _gameplay->plays())
	{
		ts << play->name() << "\n";
	}
}

void PlayConfigTab::on_goalie_toggled(bool checked)
{
	if (checked)
	{
		_gameplay->createGoalie();
	} else {
		_gameplay->removeGoalie();
	}
}

void PlayConfigTab::frameUpdate()
{
	ui.lblCurrentPlay->setText(_gameplay->playName());

	BOOST_FOREACH(shared_ptr<Play> play, _gameplay->plays())
	{
		QTreeWidgetItem *item = _playMap[play->name()];
		item->setIcon(1, play->applicable() ? _iconRun : _iconWait);
	}
}

void PlayConfigTab::on_plays_itemChanged(QTreeWidgetItem* item)
{
	shared_ptr<Play> play = play_for_item(item);
	if (!play)
	{
		return;
	}
	
	if (item->checkState(0) == Qt::Checked)
	{
		_gameplay->enablePlay(play);
	} else {
		_gameplay->disablePlay(play);
	}
}

void PlayConfigTab::on_plays_customContextMenuRequested(const QPoint& pos)
{
	QTreeWidgetItem *item = ui.plays->itemAt(pos);
	
	QMenu menu;
	QAction *none = menu.addAction("None");
	QAction *single = menu.addAction("Only this");
	QAction *cat_on = 0, *cat_off = 0;
	if (item && item->childCount())
	{
		cat_on = menu.addAction("Select category");
		cat_off = menu.addAction("Deselect category");
	}
	
	QAction *act = menu.exec(ui.plays->mapToGlobal(pos));
	if (act == none)
	{
		selectNone();
	} else if (act == single)
	{
		selectNone();
		if (item->childCount())
		{
			// This is a category
			for (int i = 0; i < item->childCount(); ++i)
			{
				QTreeWidgetItem *child = item->child(i);
				child->setCheckState(0, Qt::Checked);
			}
		} else {
			// This is a play
			item->setCheckState(0, Qt::Checked);
		}
	} else if (cat_on && act == cat_on)
	{
		for (int i = 0; i < item->childCount(); ++i)
		{
			QTreeWidgetItem *child = item->child(i);
			child->setCheckState(0, Qt::Checked);
		}
	} else if (cat_off && act == cat_off)
	{
		for (int i = 0; i < item->childCount(); ++i)
		{
			QTreeWidgetItem *child = item->child(i);
			child->setCheckState(0, Qt::Unchecked);
		}
	}
}

void PlayConfigTab::selectNone()
{
	BOOST_FOREACH(QTreeWidgetItem *item, _playMap)
	{
		item->setCheckState(0, Qt::Unchecked);
	}
}
