#ifndef __INDEX_H__
#define __INDEX_H__

#include <iostream>
#include <string>
#include <vector>

namespace perls
{
    // Index describes a hierarchy --- a tree of Index's
    // --- operator() (key) returns the child Index object with label key
    // --- operator[] (key) returns the indices of contained under the label
    //     key
    // --- each Index 1. describes a continuous block of indices so that
    //                      index[key0] 
    //                   returns a sequence of continuous integers, as does
    //                      index(key0)[key1]
    //                2. points to its parent and children
    // --- the index is built for example: 
    //     index ----- base ------------------------ xy ----- x 0
    //             |                                      |-- y 1
    //             |
    //             |-- extras --- delayed --- D2 --- xy ----- x 2
    //                                     |              |-- y 3
    //                                     |
    //                                     |- D1 --- xy ----- x 4
    //                                                    |-- y 5
    //      perls::Index index;
    //      
    //      index.add_index ("base");
    //      index("base").add ("xy", 2);
    //
    //      index.add_index ("extras");
    //      index("extras").add ("delayed");
    //
    //      index("extras")("delayed").add ("base", "D1");
    //      index("extras")("delayed").add ("base", "D2");
    //
    //      std::cout << index[];                          // index 0 1 2 3 4 5
    //      std::cout << index["base"];                    // index 0 1
    //      std::cout << index["extras"];                  // index 2 3 4 5
    //      std::cout << index("extras")["delayed"];       // index 2 3 4 5
    //      std::cout << index("extras")("delayed")["D1"]; // index 4 5
    //
    //      std::cout << index.labels ();                      // index: base, extras
    //      std::cout << index("extras")("delayed").labels (); // index: D2, D1
    //

    class Index
    {
      public:
        Index ();
        Index (Index *parent, const std::string& label, int offset, int size);
        ~Index (); 

        void add (const std::string& label, int size=0);
        void prepend (const std::string& label, int size=0);
        void insert (const std::string& src, const std::string& label, int size);
        void remove (const std::string &key); // removes child with label
        void remove (unsigned int n); // removes child n
        //void remove_last () {_children.pop_back ();}

        unsigned int size () const {return _size;}
        bool empty () const {return _children.empty ();}

        std::string label () const {return _label;}

        unsigned int nchildren () const {return _children.size ();}
        const Index& child (unsigned int n) const;
        std::vector<int> child_inds (unsigned int n) const;
        std::string child_key (unsigned int n) const {return _children[n]->_label;}

        std::vector<int> operator[] (const std::string& key) const;
        const Index& operator() (const std::string& key) const;
        Index& operator() (const std::string& key);

        friend std::ostream& operator<< (std::ostream&, const Index&);

      private:
        Index *_parent;
        std::vector<Index*> _children;

        std::string _label;
        unsigned int _offset;
        unsigned int _size;

        void update_inds (const std::string& label, int size_diff);
        void update_remove_inds (std::vector<Index*>::iterator it, int size_diff);
        Index* find_child (const std::string& key) const;
        std::vector<Index*>::iterator find_child_it (const std::string& key);
        unsigned int find_child_index (const std::string& key) const;
    };

    std::ostream& operator<< (std::ostream& os, const Index& index);
    std::ostream& operator<< (std::ostream& os, std::vector<int> v);
}

#endif // __INDEX_H__
