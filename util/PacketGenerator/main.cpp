#include <QDir>
#include <QFile>
#include <QDebug>

#include "Packet.hpp"

#include <QDomDocument>

void parseFile(QFileInfo& info)
{
	qDebug() << "Processing file: " << info.fileName();
		
	QDomDocument doc("Packet");
	QFile file(info.canonicalFilePath());
		
	if (!file.open(QIODevice::ReadOnly))
	{
		qDebug() << "\tUnable to open file";
		return;
	}
	
	QString err;
	if (!doc.setContent(&file, &err))
	{
		qDebug() << "\tUnable to set document content: " << err;
	}
	file.close();
		
	Packet p = Packet::parse(doc);
		
	QFile out(info.baseName());
	out.open(QIODevice::WriteOnly);
	StyleStream oStream(&out);
		
	p.print(oStream);
		
	out.close();
}

void usage(const char* prog)
{
	printf("%s <dir | file>\n", prog);
	printf("\tdir: directory with .xml packet files\n");
	printf("\tfile: an xml packet definition\n");
	
	//printing usage means failure
	exit(-1);
}

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		usage(argv[0]);
	}
	
	QFileInfo fileInfo(argv[1]);
	
	if (!fileInfo.exists())
	{
		printf("File or Directory '%s' does not exist\n", argv[1]);
		return -1;
	}
	
	if (fileInfo.isFile())
	{
		//process just one file
		parseFile(fileInfo);
	}
	else if (fileInfo.isDir())
	{
		//process directory
		QDir packetDir = fileInfo.dir();
	
		QFileInfoList files = packetDir.entryInfoList(QStringList("*.xml"));
	
		Q_FOREACH(QFileInfo fileInfo, files)
		{
			parseFile(fileInfo);
		}
	}
	
	return 0;
}
