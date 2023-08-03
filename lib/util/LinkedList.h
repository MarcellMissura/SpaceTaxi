#ifndef LINKEDLIST_H_
#define LINKEDLIST_H_
#include <QDataStream>
#include <QDebug>

// This is a memory preserving LinkedList implementation designed for robot control
// applications where typically the list is often cleared and refilled with new data.
// Since the allocation of memory is a performance bottleneck, memory allocation is
// avoided in a way that heap memory is allocated only when needed for new elements that
// exceed the capacity of the list, but the memory is never released. When the list is,
// cleared only its size is reset to zero, but the allocated memory is not released.
// It is reused for faster insertion of future elements instead. This way, the capacity
// of the list only ever grows and never shrinks.
//
// The LinkedList offers a std library-compatible interface to access, add, and remove
// items at the begining and the end of the list. To navigate the list, use the iterator
// interface begin() and end() to obtain a ListIterator that you can then use to step
// through the list in forward (next()) and backward (prev()) directions. The ListIterator
// is also able to wrap around if you need to loop over all items multiple times.
// Random access is not supported.

/* copy paste snippet
LinkedList<Node> nodes;
nodes << Node();
ListIterator<Node> it = nodes.begin();
while (it.hasNext())
{
    Node& node = it.next(); // you can take a copy or a reference
    // do something with node
}

*/

template <typename T>
class LinkedList;

template <typename T>
class Vector;

// One item of the linked list with pointers to the next and the previous element in the list.
template <typename T>
struct ListItem
{
    T d;
    ListItem<T>* next=0;
    ListItem<T>* prev=0;
};

// This iterator allows you to iterate through the list from head to tail and even to cycle
// through the list over and over again. hasNext() will be false when the last item in the
// list is reached, but next() can still be called and it will reset to the head.
template <typename T>
class ListIterator
{
    friend class LinkedList<T>;

    ListItem<T>* *head=0;
    ListItem<T>* *tail=0;
    ListItem<T>* cur_=0;
    bool flipped=false;

public:

    // Returns true if this iterator is empty, i.e. has no list to iterate over.
    // This is usually only the case after it has been default constructed.
    bool isNull() const
    {
        return head==0 && tail==0 && cur_==0;
    }

    // Tells you if the iterator has one more element to move forward to.
    // Note that when hasNext() is false, through the last next() call the
    // itetrator has already flipped to the head.
    bool hasNext() const
    {
        return !flipped;
    }

    // Tells you if the iterator has one more element to move backward to.
    // When hasPrev() is false, the last prev() call already returned the
    // head of the list and the iterator has flipped to the tail.
    bool hasPrev() const
    {
        return !flipped;
    }

    // Returns true if the iterator is pointing at the tail. hasNext() is
    // still true. The next next() call on the iterator will return the
    // last element in the list, the iterator will flip to the head, and
    // hasNext() will be false.
    bool atEnd() const
    {
        return (cur_ == *tail);
    }

    // Returns true if the iterator is pointing at the head. hasPrev() is
    // still true. The next prev() call on the iterator will return the head,
    // the iterator will flip to the tail and hasPrev() will be false.
    bool atBegin() const
    {
        return (cur_ == *head);
    }

    // Returns the current element by reference and moves forward to the next one.
    // If the current element is the last element in the list (the tail), the iterator
    // is automatically reset to the head and the next hasNext() call returns false.
    // You can still call next() though and continue to itearte through the list.
    T& next()
    {
        if (cur_ == *tail)
        {
            cur_ = *head;
            flipped = true;
            return (*tail)->d;
        }
        else
        {
            cur_ = cur_->next;
            flipped = false;
            return cur_->prev->d;
        }
    }

    // Returns the current element and moves the iterator backward to the previous one.
    // If the current one is the first element in the list, the iterator is reset to the tail.
    T& prev()
    {
        if (cur_ == *head)
        {
            cur_ = *tail;
            flipped = true;
            return (*head)->d;
        }
        else
        {
            cur_ = cur_->prev;
            flipped = false;
            return cur_->next->d;
        }
    }

    // Returns the current element without modifying the iterator.
    T& peekCur() const
    {
        return cur_->d;
    }

    // Returns the current element without modifying the iterator.
    T& cur() const
    {
        return cur_->d;
    }

    // Returns the previous element without modifying the iterator.
    T& peekPrev() const
    {
        if (cur_ == *head)
            return (*tail)->d;
        return cur_->prev->d;
    }

    // Returns the next element without modifying the iterator.
    T& peekNext() const
    {
        if (cur_ == *tail)
            return (*head)->d;
        return cur_->next->d;
    }

    // Returns an iterator pointing to the previous element.
    ListIterator<T> prevIt() const
    {
        ListIterator<T> ret = *this;
        ret.prev();
        return ret;
    }

    // Returns an iterator pointing to the next element.
    ListIterator<T> nextIt() const
    {
        ListIterator<T> ret = *this;
        ret.next();
        return ret;
    }

    // Resets the iterator to the head of the list.
    void reset()
    {
        cur_ = *head;
        flipped = false;
    }

    // Returns true if the other and this ListIterator point at the same item in the same LinkedList.
    bool operator==(const ListIterator<T>& other) const
    {
        return cur_ == other.cur_;
    }

    // Returns false if the other and this ListIterator point at the same item in the same LinkedList.
    bool operator!=(const ListIterator<T>& other) const
    {
        return !(*this == other);
    }

    // Check if this iterator points to a neighbouring element (prev or next) of the other iterator.
    bool isNeighbour(const ListIterator<T>& other) const
    {
        return other.cur_->next == this->cur_ || other.cur_->prev == this->cur_;
    }

    // Makes the current element of the iterator to be the head of the list such that
    // hasNext() and atEnd() will work as expected.
    void makeHead()
    {
        if (cur_ == *head)
            return;

        // Saved memory slots after the tail.
        if ((*tail)->next != 0)
            (*tail)->next->prev = cur_->prev;
        cur_->prev->next = (*tail)->next;

        // Saved memory slots before the head.
        if ((*head)->prev != 0)
            (*head)->prev->next = cur_;
        ListItem<T>*  oldcurprev = cur_->prev;
        cur_->prev = (*head)->prev;

        // Attach old head to tail.
        (*tail)->next = *head;
        (*head)->prev = *tail;

        // Update the head and the tail.
        *head = cur_;
        *tail = oldcurprev;
    }
};

template <typename T>
class LinkedList
{
    mutable ListItem<T>* head; // Pointer to the first element in the list. It has to be mutable for swapping.
    mutable ListItem<T>* tail; // Pointer to the last element in the list. It has to be mutable for swapping.
    uint size_; // A counter for the number of items in the list.

public:

    LinkedList()
    {
        head = new ListItem<T>();
        tail = head;
        size_ = 0;
    }

    ~LinkedList()
    {
        ListItem<T>* cur = head->next;
        while (cur != 0)
        {
            ListItem<T>* cur2 = cur;
            cur = cur->next;
            delete cur2;
        }
        cur = head->prev;
        while (cur != 0)
        {
            ListItem<T>* cur2 = cur;
            cur = cur->prev;
            delete cur2;
        }
        delete head;
    }

    // Copy constructor.
    LinkedList(const LinkedList<T> &o)
    {
        head = new ListItem<T>();
        tail = head;
        size_ = 0;
        *this = o;
    }

    // Assignment operator.
    // Assignment of one linked list to another creates a deep copy.
    // If the list to be copied is smaller than this one, already allocated memory is preserved.
    // Otherwise this list grows in size to accomodate the other list.
    LinkedList<T>& operator=(const LinkedList<T> &o)
    {
        if (this == &o)
            return *this;

        clear();

        ListIterator<T> it = o.begin();
        while (it.hasNext())
            push_back(it.next());

        return *this;
    }

    // Vector copy constructor.
    LinkedList(const Vector<T> &o)
    {
        head = new ListItem<T>();
        tail = head;
        *this = o;
    }

    // Vector assignment operator.
    // Assigning a Vector to a LinkedList creates a deep copy. The elements of the vector are
    // pushed into the LinkedList one by one. If the vector to be copied is smaller than this
    // list, already allocated memory is preserved. Otherwise this list grows in size to
    // accomodate the vector.
    LinkedList<T>& operator=(const Vector<T>& v)
    {
        clear();
        for (uint i = 0; i < v.size(); i++)
            push_back(v[i]);
        return *this;
    }

    // Cast to Vector operator.
    //operator Vector<T>() const {return Vector<T>(*this);}

    // Returns true if all items in this and the other LinkList evaluate the == operator to true.
    // Both lists must contain the same number of elements.
    bool operator==(const LinkedList<T>& other) const
    {
        if (size() != other.size())
            return false;

        ListIterator<T> it1 = begin();
        ListIterator<T> it2 = other.begin();
        while (it1.hasNext() && it2.hasNext())
            if (it1.next() != it2.next())
                return false;
        return true;
    }

    // Returns true if this LinkedList is not of the same size as the other, or if at least one
    // item evaluates the == operator to false.
    bool operator!=(const LinkedList<T>& other) const
    {
        if (size() != other.size())
            return true;

        ListIterator<T> it1 = begin();
        ListIterator<T> it2 = other.begin();
        while (it1.hasNext() && it2.hasNext())
            if (it1.next() != it2.next())
                return true;
        return false;
    }

    // Clears the list, but does not release already allocated memory.
    void clear()
    {
        tail = head;
        size_ = 0;
    }

    // Returns the number of items in the list.
    // This is a rather expensive operation as the items have to explicitely counted.
    uint size() const
    {
        return size_;
    }

    // Tells you if the list is empty or not.
    bool isEmpty() const
    {
        return (size_ == 0);
    }

    // Prepends a new item to the front of the list. This also sets
    // the head to be the newly pushed item. Allocated slots in memory
    // at the front of the list are reused, but most likely there aren't
    // any. Pushing to the front is not recommended.
    void push_front(const T& e)
    {
        // List is empty case.
        if (isEmpty())
        {
            head->d = e;
        }

        // Reusing memory case.
        else if (head->prev != 0)
        {
            head = head->prev;
            head->d = e;
        }

        // Allocating new memory case.
        else
        {
            head->prev = new ListItem<T>();
            head->prev->next = head;
            head = head->prev;
            head->d = e;
        }

        size_++;
    }

    // Appends a new item to the back of the list.
    // This is a very fast operation when the capacity exceeds the size.
    void push_back(const T& e)
    {
        // List is empty case.
        if (isEmpty())
        {
            head->d = e;
        }

        // Reusing memory case.
        else if (tail->next != 0)
        {
            tail = tail->next;
            tail->d = e;
        }

        // Allocating new memory case.
        else
        {
            tail->next = new ListItem<T>();
            tail->next->prev = tail;
            tail = tail->next;
            tail->d = e;
        }

        size_++;
    }

    // Appends a new item to the back of the list.
    LinkedList<T>& operator<<(const T& e)
    {
        push_back(e);
        return *this;
    }

    // Appends a new item to the back of the list.
    LinkedList<T>& operator<<(const Vector<T>& e)
    {
        for (uint i = 0; i < e.size(); i++)
            push_back(e[i]);
        return *this;
    }

    // Appends a new item to the back of the list.
    LinkedList<T>& operator<<(const LinkedList<T>& e)
    {
        ListIterator<T> it = e.begin();
        while (it.hasNext())
            push_back(it.next());
        return *this;
    }

    // Appends a new item to the back of the list.
    void push(const T& e)
    {
        push_back(e);
    }

    // Returns and removes the first item from the list.
    T pop_front()
    {
        // Empty list case.
        if (isEmpty())
            return T();

        // One item case.
        if (head == tail)
        {
            size_ = 0;
            return head->d;
        }

        head = head->next;
        size_--;
        return head->prev->d;
    }

    // Returns and removes the last item from the List.
    T pop_back()
    {
        // Empty list case.
        if (isEmpty())
            return T();

        // One item case.
        if (head == tail)
        {
            size_ = 0;
            return head->d;
        }

        tail = tail->prev;
        size_--;
        return tail->next->d;
    }

    // Returns and removes the last item from the List.
    T pop()
    {
        return pop_back();
    }

    // Returns true if the list contains at least one element that evaluates
    // the == operator to true with the given element.
    bool contains(const T& d) const
    {
        // Empty list case.
        if (isEmpty())
            return false;

        ListIterator<T> it = begin();
        while (it.hasNext())
            if (it.next() == d)
                return true;

        return false;
    }

    // Returns true if this list contains all elements of list o, i.e. the == operator
    // evaluates to true for all elements of o at least with one element of this list.
    bool contains(const LinkedList<T>& o) const
    {
        // Empty list case.
        if (isEmpty())
            return false;

        ListIterator<T> it = o.begin();
        while (it.hasNext())
            if (!contains(it.next()))
                return false;
        return true;
    }

    // Returns true if this list contains all elements of Vector o, i.e. the == operator
    // evaluates to true for all elements of o at least with one element of this list.
    bool contains(const Vector<T>& o) const
    {
        // Empty list case.
        if (isEmpty())
            return false;

        for (uint i = 0; i < o.size(); i++)
            if (!contains(o[i]))
                return false;
        return true;
    }

    // Unifies this LinkedList with item o in a way that the item appears only once in the list.
    // o is pushed only if it is not already contained.
    void unify(const T& o)
    {
        if (!contains(o))
            push_back(o);
        return;
    }

    // Unifies this LinkedList with Vector o in a way that any element of o appears only once in the result.
    void unify(const Vector<T>& o)
    {
        for (uint i = 0; i < o.size(); i++)
            if (!contains(o[i]))
                push_back(o[i]);
        return;
    }

    // Unifies this LinkedList with LinkedList o in a way that any element of o appears only once in the result.
    void unify(const LinkedList<T>& o)
    {
        ListIterator<T> it = o.begin();
        while (it.hasNext())
        {
            if (!contains(it.cur()))
                push_back(it.cur());
            it.next();
        }
        return;
    }

    // Appends a list of new elements onto the end of the linked list.
    // The elements are copied by iterating from "from" to "to" including the elements at "from" and "to".
    void append(const ListIterator<T>& from, const ListIterator<T>& to)
    {
        ListIterator<T> localFrom = from;
        while (localFrom != to)
            push_back(localFrom.next());
        push_back(localFrom.next());
    }

    // Inserts a new element into the linked list before the position indicated by the iterator it.
    // The iterator remains unmodified and can be still used after calling this function.
    void insert(const ListIterator<T>& it, const T& e)
    {
        // List is empty case.
        if (isEmpty())
            push_back(e);

        else if (it.atBegin())
            push_front(e);

        // Reusing memory case.
        else if (tail->next != 0)
        {
            tail->next->d = e;
            it.cur_->prev->next = tail->next;
            tail->next = tail->next->next;
            if (tail->next != 0)
                tail->next->prev = tail;
            it.cur_->prev->next->prev = it.cur_->prev;
            it.cur_->prev->next->next = it.cur_;
            it.cur_->prev = it.cur_->prev->next;
            size_++;
        }

        // Allocating new memory case.
        else
        {
            it.cur_->prev->next = new ListItem<T>();
            it.cur_->prev->next->d = e;
            it.cur_->prev->next->prev = it.cur_->prev;
            it.cur_->prev->next->next = it.cur_;
            it.cur_->prev = it.cur_->prev->next;
            size_++;
        }
    }

    // Inserts a list of new elements into the linked list before the position indicated by the iterator "at".
    // The elements in question are copied to this list and the argument list e remains unmodified. The iterator
    // "at" remains untouched and can still be used after calling this function.
    void insert(const ListIterator<T>& at, const LinkedList<T>& e)
    {
        ListIterator<T> eit = e.begin();
        while (eit.hasNext())
            insert(at, eit.next());
        insert(at, eit.next());
    }

    // Inserts a list of new elements into the linked list before the position indicated by the iterator "at".
    // The elements are copied by iterating from "from" to "to" including the elements at "from" and "to".
    void insert(const ListIterator<T>& at, const ListIterator<T>& from, const ListIterator<T>& to)
    {
        ListIterator<T> localFrom = from;
        while (localFrom != to)
            insert(at, localFrom.next());
        insert(at, localFrom.next());
    }

    // Replaces the elements from including "thisFrom" to including "thisTo" in this list with the elements from
    // "otherFrom" to "otherTo" also including the boundaries. The elements are copied and the sequence from
    // "otherFrom" to "otherTo" remains untouched. The memory of the replaced elements is recycled.
    void replace(const ListIterator<T>& thisFrom, const ListIterator<T>& thisTo, const ListIterator<T>& otherFrom, const ListIterator<T>& otherTo)
    {
        ListIterator<T> localThisTo = thisTo;
        bool atEnd = thisTo.atEnd();
        localThisTo.next();
        remove(thisFrom, thisTo);
        if (atEnd)
            append(otherFrom, otherTo);
        else
            insert(localThisTo, otherFrom, otherTo);
    }

    // Swaps the elements from including "thisFrom" to including "thisTo" with the elements from "otherFrom"
    // to "otherTo". Both lists are modified. Swapping is the fastest way of splicing linked lists as the swap
    // can be done in constant time, no matter how long the sequences are. Be aware that memory is swapped
    // between this and the other list and by providing faulty iterators, it is possible to mess things up in
    // ways that are worse than a crash.
    void swap(const ListIterator<T>& thisFrom, const ListIterator<T>& thisTo, const ListIterator<T>& otherFrom, const ListIterator<T>& otherTo)
    {
        if (thisFrom.atBegin())
            *thisFrom.head = otherFrom.cur_;

        if (thisTo.atEnd())
            *thisTo.tail = otherTo.cur_;

        if (otherFrom.atBegin())
            *otherFrom.head = thisFrom.cur_;

        if (otherTo.atEnd())
            *otherTo.tail = thisTo.cur_;

        if (thisFrom.cur_->prev != 0)
            thisFrom.cur_->prev->next = otherFrom.cur_;
        if (thisTo.cur_->next != 0)
            thisTo.cur_->next->prev = otherTo.cur_;

        if (otherFrom.cur_->prev != 0)
            otherFrom.cur_->prev->next = thisFrom.cur_;
        if (otherTo.cur_->next != 0)
            otherTo.cur_->next->prev = thisTo.cur_;

        ListItem<T>* tmp = thisFrom.cur_->prev;
        thisFrom.cur_->prev = otherFrom.cur_->prev;
        otherFrom.cur_->prev = tmp;
        tmp = thisTo.cur_->next;
        thisTo.cur_->next = otherTo.cur_->next;
        otherTo.cur_->next = tmp;
    }

    // Removes all elements from the linked list that evaluate the == operator
    // to true with the given element d. This is an O(N) operation.
    void removeAll(const T& d)
    {
        // Empty list case.
        if (isEmpty())
            return;

        ListIterator<T> it = begin();
        while (it.hasNext())
        {
            if (it.peekCur() == d)
                remove(it);
            else
                it.next();
        }
    }

    // Removes the first element from the linked list that evaluates the == operator
    // to true with the given element d. This is an O(N) operation.
    void removeOne(const T& d)
    {
        // Empty list case.
        if (isEmpty())
            return;

        ListIterator<T> it = begin();
        while (it.hasNext())
        {
            if (it.peekCur() == d)
            {
                remove(it);
                return;
            }
            else
            {
                it.next();
            }
        }
    }

    // Removes the element from the linked list that is indicated by the iterator "it".
    // The memory of the removed slot is saved by appending it invisibly to the end.
    // The iterator is automatically updated to point at the next element after the
    // removed one. If you remove the head, the subsequent element in the list will
    // become the head and the iterator points at this new head. If you remove the tail,
    // the previous item in the list becomes the new tail and the iterator points at
    // the head. This is a fast O(1) operation. All other iterators of the same instance
    // of LinkedList are invalidated.
    void remove(ListIterator<T>& it)
    {
        // Empty list case.
        if (isEmpty())
            return;

        if (it.atEnd())
        {
            pop_back();
            it.cur_ = head;
            it.flipped = true;
            return;
        }

        if (it.atBegin())
        {
            pop_front();
            it.cur_ = head;
            return;
        }

        ListIterator<T> nextIt = it.nextIt();

        // Remove the slot.
        it.cur_->prev->next = it.cur_->next;
        it.cur_->next->prev = it.cur_->prev;

        // Append the removed slot to the end to preserve the memory.
        ListItem<T>* end = tail;
        while (end->next != 0)
            end = end->next;
        end->next = it.cur_;
        end->next->prev = end;
        end->next->next = 0;

        it = nextIt; // Update the iterator.

        size_--;
    }

    // Removes the elements from the linked list between from and to inclusively.
    // If the head and/or the tail are removed, they are set to the nearest item.
    // The memory of the removed slots is saved by appending it invisibly to the end.
    // Both iterators are automatically updated to point at the next element after to,
    // or at the tail if to was the last element. to must be reachable by next()
    // operators from from or something undefined will happen.
    void remove(ListIterator<T>& from, ListIterator<T>& to)
    {
        // Empty list case.
        if (isEmpty())
            return;

        if (from.atBegin() && to.atEnd())
        {
            clear();
            from.cur_ = head;
            to.cur_ = head;
            return;
        }

        if (from.atEnd())
        {
            pop_back();
            from.cur_ = tail;
            to.cur_ = tail;
            return;
        }

        if (to.atBegin())
        {
            pop_front();
            from.cur_ = head;
            to.cur_ = head;
            return;
        }

        // This would be a fast operation if not for the size.
        ListIterator<T> fromIt = from;
        size_--;
        while (fromIt != to)
        {
            fromIt.next();
            size_--;
        }

        if (from.atBegin())
        {
            head = to.cur_->next;
            from.cur_ = head;
            to.cur_ = head;
        }

        else if (to.atEnd())
        {
            tail = from.cur_->prev;
            from.cur_ = tail;
            to.cur_ = tail;
        }

        else
        {
            // Remove all slots with one operation.
            from.cur_->prev->next = to.cur_->next;
            to.cur_->next->prev = from.cur_->prev;

            // Append the removed slots to the end to preserve the memory.
            ListItem<T>* end = tail;
            while (end->next != 0)
                end = end->next;
            end->next = from.cur_;
            end->next->prev = end;
            to.next();
            from = to;
            end->next->next = 0;
        }
    }

    // Removes one item from the linked list specified by the pointer p.
    // This is an O(1) operation, but it isn't safe as you are responsible
    // for passing a correct pointer to an element that is actually in the
    // list. If you provide a wrong pointer, the result is undefined.
    // If you remove the head, the subsequent element in the list will
    // become the head. If you remove the tail, the previous item in the list
    // becomes the new tail.
    void remove(T* p)
    {
        // Empty list case.
        if (isEmpty())
            return;

        ListItem<T>* pp = (ListItem<T>*)p;

        if (pp == head)
        {
            pop_front();
            return;
        }

        if (pp == tail)
        {
            pop_back();
            return;
        }

        // Remove the slot.
        pp->prev->next = pp->next;
        pp->next->prev = pp->prev;

        // Append the removed slot to the end to preserve the memory.
        ListItem<T>* end = tail;
        while (end->next != 0)
            end = end->next;
        end->next = pp;
        end->next->prev = end;
        end->next->next = 0;

        size_--;
    }

    // Returns the first element in the list (the head) without modifying the list.
    T& first() const
    {
        return head->d;
    }

    // Returns the last elemet in the list (the tail) without modifying the list.
    T& last() const
    {
        return tail->d;
    }

    // The same as last().
    T& top() const
    {
        return last();
    }

    // Returns an iterator pointing at the head of the list.
    ListIterator<T> begin() const
    {
        ListIterator<T> it;
        it.head = &(head); // Taking a pointer to the head pointer here makes the iterator more robust to changes of the list while iterating over it.
        it.tail = &(tail);
        it.cur_ = head;
        it.flipped = isEmpty();
        return it;
    }

    // Returns an iterator pointing at the end of the list.
    ListIterator<T> end() const
    {
        ListIterator<T> it;
        it.head = &(head);
        it.tail = &(tail);
        it.cur_ = tail;
        it.flipped = isEmpty();
        return it;
    }

    // Returns an iterator pointing at the element p specified by the pointer.
    // This is an unsafe operation. You are responsible for passing a correct
    // pointer to an element that is actually in the list. If you provide a wrong
    // pointer, the result is undefined.
    ListIterator<T> iteratorAt(T* p) const
    {
        ListIterator<T> it;
        it.head = &(head);
        it.tail = &(tail);
        it.cur_ = (ListItem<T>*)p;
        it.flipped = isEmpty();
        return it;
    }

    // Sorts the list in ascending order using the < operator, i.e. if a < b, then a
    // comes before b. If you pass a -1 as the argument, it sorts in reverse order.
    void sort(int direction=0)
    {
        // One item case.
        if (head == tail)
            return;

        ListItem<T> dummy;
        dummy.next = head;
        dummy.prev = head->prev;
        tail = mergesort(&dummy, size(), direction);
        head = dummy.next;
        head->prev = dummy.prev;

        // Fix the prev pointers.
        ListItem<T> *cur = head;
        while (cur != tail)
        {
            cur->next->prev = cur;
            cur = cur->next;
        }
    }

    // Reverses the order of the items in the list. The tail becomes the head and the head becomes the tail.
    // This is an O(N) operation, because all pointers need to be changed.
    void reverse()
    {
        if (head == tail)
            return;

        // Flip all prev and next pointers to invert the order.
        ListItem<T> *cur = head;
        ListItem<T> *tmp;
        while (cur != tail)
        {
            tmp = cur->next;
            cur->next = cur->prev;
            cur->prev = tmp;
            cur = cur->prev;
        }

        tmp = tail->next;
        tail->next = tail->prev;
        tail->prev = tmp;

        // Fix up the prev and next pointers of the head and the tail.
        tmp = head->next;
        head->next = tail->prev;
        if (head->next != 0)
            head->next->prev = head;
        tail->prev = tmp;
        if (tail->prev != 0)
            tail->prev->next = tail;

        // Switch head and tail.
        tmp = head;
        head = tail;
        tail = tmp;
    }

private:
    ListItem<T>* mergesort(ListItem<T> *start, long lengtho, int direction)
    {
        long count1 = (lengtho/2), count2 = (lengtho-count1);
        ListItem<T> *next1,*next2,*tail1,*tail2,*tail;
        if (lengtho<=1) return start->next;  /* Trivial case. */
        tail1 = mergesort(start, count1, direction);
        tail2 = mergesort(tail1, count2, direction);
        tail = start;
        next1 = start->next;
        next2 = tail1->next;
        tail1->next = tail2->next; /* in case this ends up as the tail */
        while (1)
        {
            if (direction+(next1->d < next2->d)) // Use of < operator.
            {
                tail->next = next1; tail = next1;
                if(--count1==0) { tail->next = next2; return tail2; }
                next1 = next1->next;
            }
            else
            {
                tail->next = next2; tail = next2;
                if(--count2==0) { tail->next = next1; return tail1; }
                next2 = next2->next;
            }
        }
    }

public:
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

template <typename T>
void LinkedList<T>::streamOut(QDataStream& out) const
{
    out << size();
    if (isEmpty())
        return;
    ListIterator<T> it = begin();
    while (it.hasNext())
        out << it.next();
}

template <typename T>
void LinkedList<T>::streamIn(QDataStream &in)
{
    clear();
    int howMany;
    in >> howMany;
    for (int i = 0; i < howMany; i++)
    {
        T e;
        in >> e;
        push_back(e);
    }
}

template <typename T>
QDataStream& operator<<(QDataStream& out, const LinkedList<T> &o)
{
    o.streamOut(out);
    return out;
}

template <typename T>
QDataStream& operator>>(QDataStream& in, LinkedList<T> &o)
{
    o.streamIn(in);
    return in;
}

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const LinkedList<T> *o)
{
    uint listSize = o->size();
    bool sp = dbg.autoInsertSpaces();
    dbg << o->size() << " [";
    if (listSize > 0)
    {
        ListIterator<T> it = o->begin();
        dbg << &it.next();
        while (it.hasNext())
            dbg << "," << &it.next();
    }
    dbg << "] ";
    dbg.setAutoInsertSpaces(sp);
    return dbg;
}

template <typename T>
QDebug operator<<(QDebug dbg, const LinkedList<T> &o)
{
    uint listSize = o.size();
    bool sp = dbg.autoInsertSpaces();
    dbg << o.size() << " [";
    if (listSize > 0)
    {
        ListIterator<T> it = o.begin();
        dbg << it.next();
        while (it.hasNext())
            dbg << "\n" << it.next();
    }
    dbg << "] ";
    dbg.setAutoInsertSpaces(sp);
    return dbg;
}

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const ListIterator<T> &o)
{
    bool sp = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << o.peekCur();
    dbg.setAutoInsertSpaces(sp);
    return dbg;
}

#endif
