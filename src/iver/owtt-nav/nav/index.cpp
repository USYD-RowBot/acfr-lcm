#include "index.h"

perls::Index::Index ()
    : _parent (0), _label (""), _offset (0), _size (0)
{}

perls::Index::Index (Index *parent, const std::string& label, int offset, int size)
    : _parent (parent), _label (label), _offset (offset), _size (size)
{}

perls::Index::~Index () 
{
    for (unsigned int i=0;i<_children.size (); ++i)
        delete _children[i];
}

void perls::Index::update_inds (const std::string& label, int size_diff)
{
    _size += size_diff;

    unsigned int updated_child = find_child_index (label);
    for (unsigned int c=updated_child+1; c<_children.size (); ++c)
        _children[c]->_offset += size_diff;

    if (_parent) {
        _parent->update_inds (_label, size_diff);
    }
}

void perls::Index::add (const std::string& label, int size)
{
    Index *child_index = new Index (this, label, _offset+_size, size);
    _children.push_back (child_index);
    _size += size;

    if (_parent) {
        _parent->update_inds (_label, size);
    }
}

void perls::Index::prepend (const std::string& label, int size)
{
    Index *child_index (new Index (this, label, _offset, size));
    _children.insert (_children.begin (), child_index);

    update_inds (label, size);
}

void perls::Index::insert (const std::string& src, const std::string& label, int size)
{
    std::vector<Index*>::iterator it = find_child_it (src);

    int offset = (*it)->_offset;
    Index *child_index = new Index (this, label, offset, size);
    _children.insert (it, child_index);

    update_inds (label, size);
}

void perls::Index::update_remove_inds (std::vector<Index*>::iterator it, int size_diff)
{
    _size -= size_diff;

    while (it != _children.end ()) {
        (*it)->_offset -= size_diff;
        it++;
    }

    if (_parent) {
        _parent->update_inds (_label, -size_diff);
    }
}

void perls::Index::remove (const std::string& key)
{
    std::vector<Index*>::iterator it = find_child_it (key);
    if (it != _children.end ()) {
        int sz = (*it)->_size;
        _children.erase (it);
        update_remove_inds (it, sz);
    }
    else 
        std::cerr << "{Index} requesting to remove child that does not exist!!!" << std::endl;
}

void perls::Index::remove (unsigned int n)
{
    if (n < _children.size ()) {
        int sz = _children[n]->_size;
        _children.erase (_children.begin() + n);
        update_remove_inds (_children.begin () + n, sz);
    }
    else 
        std::cerr << "{Index} requesting to remove child that does not exist!!!" << std::endl;
}

const perls::Index& perls::Index::child (unsigned int n) const
{
    if (n < _children.size ())
        return *_children[n];
    else {
        std::cerr << "{Index} requesting child outside of bounds!!!" << std::endl;
        return *this;
    }
}

std::vector<int> perls::Index::child_inds (unsigned int n) const
{
    Index *ind = _children[n];
    std::vector<int> v (ind->_size);
    for (unsigned int j=0;j<ind->_size;++j) v[j] = ind->_offset + j;
    return v;
}

std::vector<int> perls::Index::operator[] (const std::string& key) const
{
    std::vector<int> v;

    if (key.empty ()) {
        v.resize (_size);
        for (unsigned int j=0;j<_size;++j) v[j] = _offset + j;
        return v;
    }

    Index *ind = find_child (key);
    if (ind) {
        unsigned int sz = ind->_size;
        v.resize (sz);
        for (unsigned int j=0;j<sz;++j) v[j] = ind->_offset + j;
    }
    else {
        std::cerr << "{Index} could not find index " << key << "!!!" << std::endl;
    }
    return v;
}

perls::Index& perls::Index::operator() (const std::string& key) 
{
    Index *index = find_child (key);
    return (index ? *index : *this);
}

const perls::Index& perls::Index::operator() (const std::string& key) const
{
    Index *index = find_child (key);
    return (index ? *index : *this);
}

perls::Index* perls::Index::find_child (const std::string& key) const
{
    Index *ind = 0;
    for (unsigned int i=0;i<_children.size ();++i)
        if (_children[i]->_label == key) {
            ind = _children[i];
            break;
        }
    return ind;
}

std::vector<perls::Index*>::iterator perls::Index::find_child_it (const std::string& key) 
{
    std::vector<Index*>::iterator it;
    for (it=_children.begin (); it!=_children.end (); ++it)
        if ((*it)->_label == key) break;
    return it;
}

unsigned int perls::Index::find_child_index (const std::string& key) const
{
    for (unsigned int i=0;i<_children.size ();++i)
        if (_children[i]->_label == key) return i;
    return _children.size ();
}

std::ostream& perls::operator<< (std::ostream& os, const perls::Index& index)
{
    os << (index._label.size () ? index._label : "{Index} Index") << ": " 
        << "offset = " << index._offset 
        << ", size = " << index._size << std::endl;

    os << "Parent: " << (index._parent ? (index._parent->_label.size () ? index._parent->_label : "Index"): "Root")
        << std::endl;

    os << "Children: ";
    int nchildren = index._children.size ();
    if (!nchildren)
        os << "None";
    else 
        for (unsigned int i=0;i<index._children.size ();++i)
            os << index._children[i]->_label << ", ";
    os << std::endl;

    return os;
}

std::ostream& perls::operator<< (std::ostream& os, std::vector<int> v)
{
    for (unsigned int i=0;i<v.size ();++i)
        os << v[i] << " ";
    os << std::endl;
    return os;
}

