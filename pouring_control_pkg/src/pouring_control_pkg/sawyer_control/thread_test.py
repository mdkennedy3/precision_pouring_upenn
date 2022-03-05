import threading, time
def worker():
    """thread worker function"""
    print 'Worker'
    return
class thread_testing(object):
    """docstring for thread_testing"""
    def __init__(self):
        self.arg = 0
        self.count = 0
        self.t1 = 0
        self.t2 = 0
        self.r = threading.RLock()
    def tim(self):
        print "entered t1"
        self.r.acquire(blocking=True)
        var = self.count
        self.r.release()
        while var < 20:            
            self.t1 += 1
            self.r.acquire(blocking=True)
            self.arg += 1
            self.count += 1
            print "\ninside thread 1", " arg ", self.arg, " count: ", self.count, " t1: ", self.t1
            var = self.count
            self.r.release()
            

    def tom(self):
        print "entered t2"
        self.r.acquire(blocking=True)
        var = self.count
        self.r.release()
        while var < 20:     
            self.t2 += 1
            self.r.acquire(blocking=True)
            self.arg -= 1
            self.count += 1
            print "\ninside thread 2", " arg ", self.arg, " count: ", self.count, " t2: ", self.t2
            var = self.count
            self.r.release()





def main():
    thread_class = thread_testing()
    print "threads initialized"
    thread1  = threading.Thread(target=thread_class.tim) #can also be named: name='t1'
    thread2  = threading.Thread(target=thread_class.tom)
    # thread1  = threading.Thread(target=thread_class.t1) #can also be named: name='t1'
    # thread2  = threading.Thread(target=thread_class.t2)
    # thread1  = threading.Thread(target=worker) #can also be named: name='t1'

    print "threads started"
    thread1.start()
    thread2.start()
    # time.sleep(2)
    print "threads ended"




if __name__ == '__main__':
    main()
