

class TimeSlot():
    """A generic time slot object
    slot must have non-zero size.
    """
    
    def __init__(self, t_begin, t_end, priority=0, data=None) -> None:
        self.t_begin  = t_begin
        self.t_end    = t_end
        self.priority = priority # integer higher the better
        self.data     = data # generic data. can be anything
        if (t_begin >= t_end):
            raise ValueError('t_begin must be strictly less than t_end')
        return
    
    def __str__(self):
        return 'Time from ' + str(self.t_begin) + ' to ' + str(self.t_end) +', priority: ' + str(self.priority)

    def contains_time(self, t) -> bool:
        """determine whether t is contained within this time slot.
        Note: 
            Start time inclusive, end time exclusive"""
        return self.t_begin <= t and t < self.t_end
    def is_before(self, slot_other: 'TimeSlot') -> bool:
        """Return true if this slot comes entirely before slot_other"""
        return self.t_end <= slot_other.t_begin
    def is_after(self, slot_other: 'TimeSlot') -> bool:
        """Return true if this slot comes entirely after slot_other"""
        return slot_other.t_end <= self.t_begin
    def geq_priority(self, slot_other: 'TimeSlot') -> bool:
        """Returns true if this slot has a priority greater or equal to that of slot_other"""
        return self.priority>=slot_other.priority
    def intersects(self, slot_other: 'TimeSlot') -> bool:
        """Return true if two slots intersect"""
        return not (self.is_before(slot_other) or self.is_after(slot_other))
    def contains(self, slot_other: 'TimeSlot')->bool:
        return self.t_begin < slot_other.t_begin and self.t_end > slot_other.t_end
    def intersection(self, slot_other: 'TimeSlot') -> 'TimeSlot':
        """return an instance of TimeSlot that is intersection of this an other.
        Priority value and data are copied from higher priority time slot.
        """
        t_begin = max(self.t_begin, slot_other.t_begin)
        t_end   = min(self.t_end, slot_other.t_end)
        
        if t_begin>=t_end:
            raise ValueError('intersection does not exist')
        
        if self.priority > slot_other.priority:
            return TimeSlot(t_begin, t_end, self.priority, self.data)
        else:
            return TimeSlot(t_begin, t_end, slot_other.priority, slot_other.data)
        
    def intersection_in_place(self, slot_other: 'TimeSlot') -> None:
        """Modifies self to be the intersection of self and other
        """
        t_begin = max(self.t_begin, slot_other.t_begin)
        t_end   = min(self.t_end, slot_other.t_end)
        if t_begin>=t_end:
            raise ValueError('intersection does not exist')
        
        self.update_t_begin(t_begin)
        self.update_t_end(t_end)
        
        # overide
        if slot_other.geq_priority(self):
            self.priority = slot_other.priority
            self.data = slot_other.data
            
    def update_t_begin(self, t_begin) -> None:
        if (t_begin >= self.t_end):
            raise ValueError('t_begin must be strictly less than t_end')
        self.t_begin = t_begin
    def update_t_end(self, t_end) -> None:
        if (self.t_begin >= t_end):
            raise ValueError('t_end must be strictly larger than t_begin')
        self.t_end = t_end
        
        
class TimeTable():
    """collection of time slots"""
    
    def __init__(self):
        self.slots = [] # non-overlapping, ordered list of TimeSlot
        self.counts = 0 # cumulative count of slots added
        pass
        
    def __str__(self):
        msg = 'Time Table:\n'
        for i, slot in enumerate(self.slots):
            msg += slot.__str__()
            if i != len(self.slots)-1:
                msg += '\n'
        return msg
    
    def size(self):
        return len(self.slots)
    
    def add_slot(self, new_slot: TimeSlot) -> None:
        
        self._add_slot(new_slot, self.counts)
        
        # check for spliced events 
        for i in reversed(range(len(self.slots)-1)):
            slot_prev = self.slots[i]
            slot_next = self.slots[i+1]
            
            if slot_prev.data['_TimeTableId'] == slot_next.data['_TimeTableId'] \
                    and slot_prev.t_end == slot_next.t_begin:
                        
                # consecutive by same event
                slot_prev.update_t_end(slot_next.t_end)
                self.slots.pop(i+1) # 
            
        self.counts += 1
        return
    
    def add_slots(self, new_slots) -> None:
        for new_slot in new_slots:
            self.add_slot(new_slot)
    
    def _add_slot(self, new_slot: TimeSlot, idx: int) -> None:
        """new slots may have overlap
        When priority is equal between the existing slot and new slot, new slot will replace the existing slot.
        """
        
        if new_slot.data is None:
            new_slot.data = {}
            
        new_slot.data['_TimeTableId'] = idx # internal variable used to track TimeSlot
        
        # no slots exists previously
        if len(self.slots) == 0:
            self.slots.append(new_slot)
            return
        
        # find new_slot point
        # strategy is to visit slots in reverse order and insert as you visit in one pass. 
        for i, slot in reversed(list(enumerate(self.slots))):
            
            if slot.t_end < new_slot.t_end:
                # 'new_slot' exceeds 'slot' to the RIGHT
                
                if slot.t_end <= new_slot.t_begin:
                    # case: slot.t_begin < slot.t_end <= new_slot.t_begin < new_slot.t_end
                    self.slots.insert(i+1, new_slot)
                    return
                else:
                    # case: slot.t_begin < new_slot.t_begin < slot.t_end < new_slot.t_end
                    # or    new_slot.t_begin < slot.t_begin < slot.t_end < new_slot.t_end
                    #
                    # in other words, there exists non-zero overlap between new slot and slot
                    slot_excess = TimeSlot(slot.t_end, new_slot.t_end, new_slot.priority, new_slot.data)
                    self.slots.insert(i+1, slot_excess)
                    new_slot.update_t_end(slot.t_end) # trim new_slot so right hand does not exceed any more
                    
            assert(new_slot.t_end <= slot.t_end) # sanity check
            
            
            if slot.t_begin < new_slot.t_end: 
                # slot and new_slot intersects! How does it intersect?
                
                # right hand part
                if new_slot.t_end < slot.t_end:
                    # non-zero space to RIGHT
                    # |<-------- slot --------->|
                    #          new_slot --->|
                    self.slots.insert(i+1, TimeSlot(new_slot.t_end, slot.t_end, slot.priority, slot.data))
                
                # resolve intersection 
                if slot.t_begin == new_slot.t_begin:
                    # |<-------- slot --------->|
                    # |<---- new_slot 
                    slot.intersection_in_place(new_slot)
                    return 
                elif slot.t_begin < new_slot.t_begin:
                    # non-zero space to LEFT
                    # |<-------- slot --------->|
                    #     |<---- new_slot 
                    slot_middle = slot.intersection(new_slot)
                    self.slots.insert(i+1, slot_middle)
                    slot.update_t_end(new_slot.t_begin)
                    return
            
                # new_slot exceeds slot to LEFT
                #           |<-------- slot --------->|
                #   |<------ new_slot --------------->|
                slot.intersection_in_place(new_slot)
                new_slot.update_t_end(slot.t_begin)
                    
            else: 
                continue # skip
        
        # reach here iff new_slot is before all the time slots
        self.slots.insert(0, new_slot)
        
        return

    def delete_old_slots(self, t) -> None:
        
        n = self.size()
        for i in range(n):
            if self.slots[0].t_end <= t:
                self.slots.pop(0)
            else:
                return
    
    def get_slot(self, t)->TimeSlot:
        """"""
        for slot in self.slots:
            if slot.t_begin <= t and t <= slot.t_end:
                return slot
        
        raise NotImplementedError('todo: how to handle when no slot exists for t?')
    
###################################
# tests
###################################

def test_TimeSlot():
    """Perform unit test for TimeSlot"""
    
    ts1 = TimeSlot(0, 10, priority=3)
    ts2 = TimeSlot(5, 15)
    ts3 = TimeSlot(2,8)
    ts4 = TimeSlot(20, 30, priority=5)
    ts5 = TimeSlot(0,10,0, {}) # pass empty data
    
    assert( ts1.contains_time(3) )
    assert( not ts1.contains_time(-5) )
    assert( not ts1.contains_time(15) )
    assert( ts1.contains(ts3) )
    assert( not ts1.contains(ts2 ) )
    assert( ts1.intersects(ts2) )
    assert( ts1.is_before(ts4) )
    assert( ts4.is_after(ts1) )
    assert( ts4.geq_priority(ts1))
    print(ts1)
    print('TimeSlot passed all the tests!')
    pass

def test_TimeTable():
    
    print("")
    ts1 = TimeSlot(0, 10, priority=1)
    ts5 = TimeSlot(0, 10, priority=0)
    ts2 = TimeSlot(5, 15, priority=3)
    ts3 = TimeSlot(-10,8)
    ts4 = TimeSlot(-5, 30, priority=5)

    time_table = TimeTable()
    
    time_table.add_slot(ts1)
    print(time_table)
    assert(time_table.size() == 1)
    
    time_table.add_slot(ts5)
    print(time_table)
    assert(time_table.size() == 1)
    
    time_table.add_slot(ts2)
    print(time_table)
    assert(time_table.size() == 2)
    
    time_table.add_slot(ts3)
    print(time_table)
    assert(time_table.size() == 3)
    
    time_table.add_slot(ts4)
    print(time_table)
    assert(time_table.size() == 2)

    print('TimeTable passed all the tests!')
    
    ts6 = TimeSlot(float('-inf'), float('inf'), priority = 0)
    ts7 = TimeSlot(0, 10, priority=1)
    time_table2 = TimeTable()
    time_table2.add_slot( ts6 )
    time_table2.add_slot( ts7 )
    print(time_table2)
    
def test_TimeTable_delete_old_slots():
    
    print("")
    ts1 = TimeSlot(0, 5,  priority=1)
    ts2 = TimeSlot(4, 8,  priority=2)
    ts3 = TimeSlot(6, 10, priority=3)

    time_table = TimeTable()
    time_table.add_slots([ts1, ts2, ts3])
    print(time_table)
    assert(time_table.size()==3)
    time_table.delete_old_slots(4)
    print(time_table)
    assert(time_table.size()==2)
    
if __name__ == '__main__':
    test_TimeSlot()
    test_TimeTable()
    test_TimeTable_delete_old_slots()