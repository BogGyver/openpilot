#!/usr/bin/env python3

# 
# https://github.com/balena/python-pqueue
# 
# Some updates by Raf for tinklad.py 5/2019
#

"""A single process, persistent multi-producer, multi-consumer queue."""

import os
import pickle
import tempfile
import shutil


from queue import Queue as SyncQ


def _truncate(fn, length):
    fd = os.open(fn, os.O_RDWR)
    os.ftruncate(fd, length)
    os.close(fd)


class Queue(SyncQ):
    def __init__(self, path, maxsize=0, chunksize=100, tempdir=None):
        """Create a persistent queue object on a given path.

        The argument path indicates a directory where enqueued data should be
        persisted. If the directory doesn't exist, one will be created. If maxsize
        is <= 0, the queue size is infinite. The optional argument chunksize
        indicates how many entries should exist in each chunk file on disk.

        The tempdir parameter indicates where temporary files should be stored.
        The tempdir has to be located on the same disk as the enqueued data in
        order to obtain atomic operations.
        """

        self.path = path
        self.chunksize = chunksize
        self.tempdir = tempdir
        if self.tempdir:
            if os.stat(self.path).st_dev != os.stat(self.tempdir).st_dev:
                raise ValueError("tempdir has to be located "
                                 "on same path filesystem")

        SyncQ.__init__(self, maxsize)
        self.info = self._loadinfo()
        # truncate head case it contains garbage
        hnum, hcnt, hoffset = self.info['head']
        headfn = self._qfile(hnum)
        if os.path.exists(headfn):
            if hoffset < os.path.getsize(headfn):
                _truncate(headfn, hoffset)
        # let the head file open
        self.headf = self._openchunk(hnum, 'ab+')
        # let the tail file open
        tnum, _, toffset = self.info['tail']
        self.tailf = self._openchunk(tnum)
        self.tailf.seek(toffset)
        # update unfinished tasks with the current number of enqueued tasks
        self.unfinished_tasks = self.info['size']
        # optimize info file updates
        self.update_info = True

    def _init(self, maxsize):
        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def _destroy(self):
        if os.path.exists(self.path):
            shutil.rmtree(self.path)
            os.makedirs(self.path)

    def _qsize(self, len=len): # pylint: disable=redefined-builtin,arguments-differ
        return self.info['size']

    def _put(self, item):
        pickle.dump(item, self.headf)
        self.headf.flush()
        hnum, hpos, _ = self.info['head']
        hpos += 1
        if hpos == self.info['chunksize']:
            hpos = 0
            hnum += 1
            self.headf.close()
            self.headf = self._openchunk(hnum, 'ab+')
        self.info['size'] += 1
        self.info['head'] = [hnum, hpos, self.headf.tell()]
        self._saveinfo()

    def _get(self):
        tnum, tcnt, toffset = self.info['tail']
        hnum, hcnt, _ = self.info['head']
        if [tnum, tcnt] >= [hnum, hcnt]:
            return None
        data = pickle.load(self.tailf)
        toffset = self.tailf.tell()
        tcnt += 1
        if tcnt == self.info['chunksize'] and tnum <= hnum:
            tcnt = toffset = 0
            tnum += 1
            self.tailf.close()
            self.tailf = self._openchunk(tnum)
        self.info['size'] -= 1
        self.info['tail'] = [tnum, tcnt, toffset]
        self.update_info = True
        return data

    def task_done(self):
        try:
            SyncQ.task_done(self)
        except: # pylint: disable=bare-except 
            pass
        if self.update_info:
            self._saveinfo()
            self.update_info = False

    def _openchunk(self, number, mode='rb'):
        return open(self._qfile(number), mode)

    def _loadinfo(self):
        infopath = self._infopath()
        if os.path.exists(infopath):
            with open(infopath, 'rb') as f:
                info = pickle.load(f)
        else:
            info = {
                'chunksize': self.chunksize,
                'size': 0,
                'tail': [0, 0, 0],
                'head': [0, 0, 0],
            }
        return info

    def _gettempfile(self):
        if self.tempdir:
            return tempfile.mkstemp(dir=self.tempdir)
        else:
            return tempfile.mkstemp()

    def _saveinfo(self):
        tmpfd, tmpfn = self._gettempfile()
        os.write(tmpfd, pickle.dumps(self.info))
        os.close(tmpfd)
        # POSIX requires that 'rename' is an atomic operation
        os.rename(tmpfn, self._infopath())
        self._clear_old_file()

    def _clear_old_file(self):
        tnum, _, _ = self.info['tail']
        while tnum >= 1:
            tnum -= 1
            path = self._qfile(tnum)
            if os.path.exists(path):
                os.remove(path)
            else:
                break

    def _qfile(self, number):
        return os.path.join(self.path, 'q%05d' % number)

    def _infopath(self):
        return os.path.join(self.path, 'info')
