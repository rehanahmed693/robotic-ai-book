import React from 'react';
import clsx from 'clsx';
import styles from './DurationEstimator.module.css';

// Define the props interface
interface DurationEstimatorProps {
  minutes: number;
  activity?: 'reading' | 'exercise' | 'project' | 'lecture';
}

// DurationEstimator component
const DurationEstimator: React.FC<DurationEstimatorProps> = ({ 
  minutes, 
  activity = 'reading' 
}) => {
  // Define icon based on activity type
  const getActivityIcon = () => {
    switch (activity) {
      case 'exercise': return '📝';
      case 'project': return '🔨';
      case 'lecture': return '🎤';
      default: return '📖';
    }
  };

  // Define label based on activity type
  const getActivityLabel = () => {
    switch (activity) {
      case 'exercise': return 'Exercise';
      case 'project': return 'Project';
      case 'lecture': return 'Lecture';
      default: return 'Reading';
    }
  };

  return (
    <div className={clsx('margin-bottom--md', styles.durationEstimator)}>
      <div className={styles.durationContent}>
        <span className={styles.icon}>{getActivityIcon()}</span>
        <div className={styles.text}>
          <span className={styles.label}>{getActivityLabel()}:</span>
          <span className={styles.duration}>{minutes} min</span>
        </div>
      </div>
    </div>
  );
};

export default DurationEstimator;