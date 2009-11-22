#include "PlayConfigTab.hpp"
#include "PlayConfigTab.moc"

#include "gameplay/Play.hpp"

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include <QFileDialog>

using namespace std;
using namespace boost;

class PlayListModel: public QAbstractListModel
{
	public:
		PlayListModel(PlayConfigTab *config);
		
		virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
		virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;
		virtual Qt::ItemFlags flags(const QModelIndex &index) const;
		virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		virtual bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
		
		void addPlay(boost::shared_ptr<Gameplay::Play> play);
		
		const std::vector<boost::shared_ptr<Gameplay::Play> > &available() const
		{
			return _available;
		}
		
		void update();
		
	private:
		PlayConfigTab *_config;
		std::vector<boost::shared_ptr<Gameplay::Play> > _available;
};

PlayListModel::PlayListModel(PlayConfigTab *config)
{
	_config = config;
}

int PlayListModel::rowCount(const QModelIndex& parent) const
{
	return _available.size();
}

int PlayListModel::columnCount(const QModelIndex& parent) const
{
	// Checkbox and name
	return 1;
}

Qt::ItemFlags PlayListModel::flags(const QModelIndex &index) const
{
	return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
}

QVariant PlayListModel::data(const QModelIndex &index, int role) const
{
	if (index.column() != 0 || index.row() < 0 || index.row() >= (int)_available.size())
	{
		return QVariant();
	}
	
	if (role == Qt::CheckStateRole)
	{
		// Checkbox
		return _config->gameplay->playEnabled(_available[index.row()]) ? Qt::Checked : Qt::Unchecked;
	} else if (role == Qt::DisplayRole)
	{
		// Name
		return QString::fromStdString(_available[index.row()]->name());
	}
	
	return QVariant();
}

bool PlayListModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	if (index.column() == 0 && index.row() >= 0 && index.row() < (int)_available.size() && role == Qt::CheckStateRole)
	{
		if (value.toInt() == Qt::Checked)
		{
			_config->gameplay->enablePlay(_available[index.row()]);
		} else {
			_config->gameplay->disablePlay(_available[index.row()]);
		}
		dataChanged(index, index);
		
		return true;
	} else {
		return false;
	}
}

void PlayListModel::addPlay(boost::shared_ptr<Gameplay::Play> play)
{
	int n = _available.size();
	beginInsertRows(QModelIndex(), n, n);
	_available.push_back(play);
	endInsertRows();
}

void PlayListModel::update()
{
	dataChanged(index(0), index(_available.size() - 1));
}

////////

PlayConfigTab::PlayConfigTab(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);
	
	_model = new PlayListModel(this);
	ui.plays->setModel(_model);
}

PlayConfigTab::~PlayConfigTab()
{
	delete _model;
}

void PlayConfigTab::addPlay(shared_ptr<Gameplay::Play> play)
{
	_model->addPlay(play);
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
	QString playbook_name = QFileInfo(filename).baseName();
	ui.lblCurrentPlaybook->setText(tr("Current Playbook: ") + playbook_name);
	
	// clear the playbook
	on_none_clicked();

	// open the file
	string fname = filename.toStdString();
	ifstream ifs(fname.c_str());

	// Build a map from name to play
	typedef map<string, shared_ptr<Gameplay::Play> > PlayMap;
	PlayMap plays;
	BOOST_FOREACH(shared_ptr<Gameplay::Play> play, _model->available())
	{
		plays[play->name()] = play;
	}
	
	// read all of the strings
	while (ifs.good()) {
		string name;
		ifs >> name;
		if (name != "") {
			PlayMap::const_iterator i = plays.find(name);
			if (i != plays.end())
			{
 				gameplay->enablePlay(i->second);
			} else {
				cout << "Missing Play: " << name << endl;
			}
		}
	}
	ifs.close();
	_model->update();
}

void PlayConfigTab::on_save_clicked()
{
	// get a filename from the user
	QString fileName = QFileDialog::getSaveFileName(this,
		     tr("Save Playbook"), "./", tr("Playbook Files (*.pbk)"));
	string fname = fileName.toStdString() + ".pbk";

	// change the name of the current playbook
	size_t name_start = fname.find_last_of("/");
	string fname1 = fname.substr(name_start+1); // remove path
	string fname2 = fname1.substr(0, fname1.size()-4); // remove ".pbk"
	QString playbook_name = QString::fromStdString(fname2);
	ui.lblCurrentPlaybook->setText(tr("Current Playbook: ") + playbook_name);

	// open the file
	ofstream ofs(fname.c_str());

	// write the plays in order
	BOOST_FOREACH(shared_ptr<Gameplay::Play> play, gameplay->plays())
	{
		ofs << play->name() << "\n";
	}
	ofs.close();
}

void PlayConfigTab::on_all_clicked()
{
	BOOST_FOREACH(shared_ptr<Gameplay::Play> play, _model->available())
	{
		gameplay->enablePlay(play);
	}
	_model->update();
}

void PlayConfigTab::on_none_clicked()
{
	BOOST_FOREACH(shared_ptr<Gameplay::Play> play, _model->available())
	{
		gameplay->disablePlay(play);
	}
	_model->update();
}

void PlayConfigTab::on_goalie_toggled(bool checked)
{
	if (checked)
	{
		cout << "Creating goalie" << endl;
		gameplay->createGoalie();
	} else {
		cout << "Removing goalie" << endl;
		gameplay->removeGoalie();
	}
}

#if 0
void PlayConfigTab::updateCurrentPlay(QString playname) {
	ui.lblCurrentPlay->setText(tr("Current Play: ")+playname);
}

void PlayConfigTab::useGoalie(int state) {
}
#endif
