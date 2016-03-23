#ifndef MODELDOWNLOADER_H
#define MODELDOWNLOADER_H

#include "QBuffer"
#include "Q3Http"
#include "QUrl"
#include "QString"

class FileDownloader : public QObject {
    Q_OBJECT

private Q_SLOTS:
    void doneDownloadingCb() { isDone = true; }

private:
    QString modelUrl;
    bool isDone;
    void spinUntilDownloaded() { while(!isDone) sleep(1); }


public:
    FileDownloader(QObject *parent = 0) {
        this->isDone = false;
    }

    bool getBufferFromUrl(QString urlString, QBuffer* buffer) {
        QUrl url(urlString);

        Q3Http http(url.host());
        http.get(url.path(), buffer);

        connect(&http, SIGNAL(done(bool)), this, SLOT(doneDownloadingCb));

        spinUntilDownloaded();

        return true;
    }

};

#endif // MODELDOWNLOADER_H
