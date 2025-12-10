import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

// Define the props interface
interface LearningObjectivesProps {
  objectives: string[];
  style?: 'list' | 'card';
}

// LearningObjectives component
const LearningObjectives: React.FC<LearningObjectivesProps> = ({ 
  objectives, 
  style = 'list' 
}) => {
  if (!objectives || objectives.length === 0) {
    return null;
  }

  return (
    <div className={clsx(
      'margin-bottom--md',
      styles.learningObjectives,
      styles[`learningObjectives--${style}`]
    )}>
      <h3 className={styles.title}>In this lesson, you will:</h3>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objectiveItem}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningObjectives;