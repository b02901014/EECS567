/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
  // STENCIL: implement your min binary heap insert operation
  heap.push(new_element);
  var curr = heap.length - 1;
  var parent = 0;

  while(curr > 0){
    if(curr % 2) parent = (curr - 1)/2;
    else parent = (curr - 2)/2;

    if(heap[curr] < heap[parent]){
      var tmp = heap[curr];
      heap[curr] = heap[parent];
      heap[parent] = tmp; 
    }
    else break;

    curr = parent;
  }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
  // STENCIL: implement your min binary heap extract operation
  ret = heap[0];
heap[0] = heap[heap.length - 1];
heap.pop();
index =0;
leftChild = index * 2+ 1;   

while(1){
smallIndex = leftChild;
 console.log("index " + index);
 console.log("smallIndex " + smallIndex);
 if(leftChild < heap.length - 1) { // two child
     if(heap[leftChild] > heap[leftChild + 1]) { // get smaller child
         smallIndex = leftChild + 1;
     }
   if(heap[index] > heap[smallIndex]) {
         temp = heap[smallIndex];
         heap[smallIndex] = heap[index];
         heap[index] = temp;
     }
     else break;
 }
 else if(leftChild == heap.length - 1) { // one child
     if(heap[index] < heap[leftChild]) {
         temp = heap[leftChild];
         heap[leftChild] = heap[index];
         heap[index] = temp;
     }
   else break;
 }
 else { // no child
     console.log("ELSE CASE ");
     return ret;
     break;
 }
 index = smallIndex;
 leftChild = index*2 + 1; 
}

return ret;
  /*
  var len = heap.length;
  var ex = heap[0];
  heap[0] = heap[len - 1];
  heap.pop();
  len -= 1;

  //heapify
  var curr = 0;
  while(1){
    var lchild = curr * 2 + 1, rchild = curr * 2 + 2;
    var new_idx;
    if(lchild < len - 1){     
      if(heap[curr] > heap[lchild] || heap[curr] > heap[rchild]){
        if(heap[lchild] < heap[rchild]) new_idx = lchild; 
        else new_idx = rchild;
      }
      else break;
    }

    else if(lchild == len - 1){
      if(heap[curr] > heap[lchild]) new_idx = lchild;
      else break;
    }

    else break;

    var tmp = heap[new_idx];
    heap[new_idx] = heap[curr];
    heap[curr] = tmp;
    curr = new_idx;

  }

  return ex;
  */
}

// assign extract function within minheaper object

minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






