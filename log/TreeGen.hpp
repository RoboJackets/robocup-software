#pragma once

namespace Log
{
	
    template <typename T>
    void handleExtArray(QStandardItem* c1, QStandardItem* c2, T* arr, unsigned int size)
    {
        for (unsigned int i=0 ; i<size ; ++i)
        {
            QStandardItem* a = c1->child(i, 0);
			QStandardItem* b = c1->child(i, 1);
			
            if (!a)
            {
                c1->setChild(i, 0, new QStandardItem(QString::number(i)));
				c1->child(i,0)->setEditable(false);
			}
			
			if (!b)
			{
                c1->setChild(i, 1, new QStandardItem());
                c1->child(i,1)->setEditable(false);
            }
            
            handleExt(c1->child(i,0), c1->child(i,1), arr[i]);
        }

        c1->removeRows(size, c1->rowCount() - size);
    }
	
	//handle built in types
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const bool& p, const char* name=0)
    {
		c2->setText((p) ? "true" : "false");
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const double& p, const char* name=0)
    {
		c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const float& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const int8_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const int16_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const int32_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const int64_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const uint8_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const uint16_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const uint32_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const uint64_t& p, const char* name=0)
    {
        c2->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* c1, QStandardItem* c2, const std::string& p, const char* name=0)
    {
        c2->setText(QString::fromStdString(p));
    }
}
