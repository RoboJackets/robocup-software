#ifndef TREEGEN_HPP_
#define TREEGEN_HPP_

namespace Log
{
    template <typename T>
    void handleExtVector(QStandardItem* item, const std::vector< T > arr)
    {
        for (unsigned int i=item->rowCount() ; i<arr.size() ; ++i)
        {
            item->appendRow(new QStandardItem());
        }

        int i=0;
        Q_FOREACH(const T& a, arr)
        {
            handleExt(item->child(i), a);
            i++;
        }

        item->removeRows(i, item->rowCount() - i);
    }
    
    template <typename T>
    void handleExtArray(QStandardItem* item, T* arr, unsigned int size)
    {
        for (unsigned int i=0 ; i<size ; ++i)
        {
            QStandardItem* c = item->child(i, 0);
            if (!c)
            {
                c = new QStandardItem();
                item->setChild(i, 0, c);
            }
            
            handleExt(c, arr[i]);
        }

        item->removeRows(size, item->rowCount() - size);
    }
	
	//handle built in types
	static inline void handleExt(QStandardItem* item, const bool& p)
    {
        item->setText((p) ? "true" : "false");
    }
	
	static inline void handleExt(QStandardItem* item, const double& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const float& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const int8_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const int16_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const int32_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const int64_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const uint8_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const uint16_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const uint32_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
	
	static inline void handleExt(QStandardItem* item, const uint64_t& p)
    {
        item->setText(QString("%1").arg(p));
    }
}

#endif /* TREEGEN_HPP_ */
